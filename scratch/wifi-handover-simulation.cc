#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include <iostream>
#include <fstream>
#include <string>
#include <map>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiHandoverSimulation");

// Extended PacketSink to track throughput
class ThroughputSink : public PacketSink {
public:
  static TypeId GetTypeId (void);
  ThroughputSink ();
  virtual ~ThroughputSink ();

  uint64_t GetLastTotalRx() const;
  void SetLastTotalRx(uint64_t lastTotalRx);

private:
  uint64_t m_lastTotalRx;
};

TypeId
ThroughputSink::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ThroughputSink")
    .SetParent<PacketSink> ()
    .AddConstructor<ThroughputSink> ()
  ;
  return tid;
}

ThroughputSink::ThroughputSink() : m_lastTotalRx(0)
{
}

ThroughputSink::~ThroughputSink()
{
}

uint64_t
ThroughputSink::GetLastTotalRx() const
{
  return m_lastTotalRx;
}

void
ThroughputSink::SetLastTotalRx(uint64_t lastTotalRx)
{
  m_lastTotalRx = lastTotalRx;
}

// User migration handler class
class UserMigrationHandler {
public:
    UserMigrationHandler(Ptr<Node> apA, Ptr<Node> apB, NodeContainer& staNodes, NetDeviceContainer& staDevices) 
    : m_apA(apA), m_apB(apB), m_staNodes(staNodes), m_staDevices(staDevices) {
        // Map stations to their initial AP
        for (uint32_t i = 0; i < staNodes.GetN(); i++) {
            if (i < 16) {
                m_staToAp[i] = 0; // First 16 stations to AP A
            } else {
                m_staToAp[i] = 1; // Next 16 stations to AP B
            }
        }
        
        // Get AP MAC addresses
        Ptr<WifiNetDevice> apDevA = DynamicCast<WifiNetDevice>(m_apA->GetDevice(0));
        Ptr<WifiNetDevice> apDevB = DynamicCast<WifiNetDevice>(m_apB->GetDevice(0));
        
        if (apDevA && apDevB) {
            m_apAMac = Mac48Address::ConvertFrom(apDevA->GetAddress());
            m_apBMac = Mac48Address::ConvertFrom(apDevB->GetAddress());
        }
    }
    
    void ScheduleMigration(double timeInSeconds, double percentAtoB, double percentBtoA) {
        Simulator::Schedule(Seconds(timeInSeconds), &UserMigrationHandler::MigrateUsers, this, percentAtoB, percentBtoA);
    }
    
    void PrintUserDistribution(std::ostream& os) {
        uint32_t countA = 0;
        uint32_t countB = 0;
        
        for (auto const& pair : m_staToAp) {
            if (pair.second == 0) countA++;
            else countB++;
        }
        
        double time = Simulator::Now().GetSeconds();
        os << time << "," << countA << "," << countB << std::endl;
        
        NS_LOG_UNCOND("Time " << time << "s: " << countA << " users at AP A, " << countB << " users at AP B");
    }

private:
    void MigrateUsers(double percentAtoB, double percentBtoA) {
        NS_LOG_UNCOND("Migrating users: " << percentAtoB*100 << "% from A->B, " << percentBtoA*100 << "% from B->A");
        
        // Count users at each AP
        std::vector<uint32_t> usersAtA;
        std::vector<uint32_t> usersAtB;
        
        for (auto const& pair : m_staToAp) {
            if (pair.second == 0) usersAtA.push_back(pair.first);
            else usersAtB.push_back(pair.first);
        }
        
        // Calculate number of users to move
        uint32_t moveAtoB = static_cast<uint32_t>(usersAtA.size() * percentAtoB);
        uint32_t moveBtoA = static_cast<uint32_t>(usersAtB.size() * percentBtoA);
        
        NS_LOG_UNCOND("Moving " << moveAtoB << " users from A->B and " << moveBtoA << " users from B->A");
        
        // Move users from A to B
        for (uint32_t i = 0; i < moveAtoB && i < usersAtA.size(); i++) {
            uint32_t staIndex = usersAtA[i];
            Ptr<WifiNetDevice> staDev = DynamicCast<WifiNetDevice>(m_staDevices.Get(staIndex));
            
            if (staDev) {
                staDev->GetMac()->SetBssid(m_apBMac, 0); // Added linkId=0
                m_staToAp[staIndex] = 1; // Now associated with AP B
                NS_LOG_UNCOND("Station " << staIndex << " moved from AP A to AP B");
            }
        }
        
        // Move users from B to A
        for (uint32_t i = 0; i < moveBtoA && i < usersAtB.size(); i++) {
            uint32_t staIndex = usersAtB[i];
            Ptr<WifiNetDevice> staDev = DynamicCast<WifiNetDevice>(m_staDevices.Get(staIndex));
            
            if (staDev) {
                staDev->GetMac()->SetBssid(m_apAMac, 0); // Added linkId=0
                m_staToAp[staIndex] = 0; // Now associated with AP A
                NS_LOG_UNCOND("Station " << staIndex << " moved from AP B to AP A");
            }
        }
        
        // Record current distribution to file
        std::ofstream userDist("user_distribution.csv", std::ios_base::app);
        PrintUserDistribution(userDist);
    }
    
    Ptr<Node> m_apA;
    Ptr<Node> m_apB;
    NodeContainer& m_staNodes;
    NetDeviceContainer& m_staDevices;
    std::map<uint32_t, int> m_staToAp; // Maps station index to AP (0=A, 1=B)
    Mac48Address m_apAMac;
    Mac48Address m_apBMac;
};

// Monitor throughput function using our custom ThroughputSink
void MonitorThroughput(Ptr<ThroughputSink> sinkA, Ptr<ThroughputSink> sinkB, std::ofstream& os, double interval) {
    double timeNow = Simulator::Now().GetSeconds();
    
    // Calculate throughput in Mbps
    double throughputA = (sinkA->GetTotalRx() - sinkA->GetLastTotalRx()) * 8.0 / interval / 1000000;
    double throughputB = (sinkB->GetTotalRx() - sinkB->GetLastTotalRx()) * 8.0 / interval / 1000000;
    
    // Save current received bytes for next calculation
    sinkA->SetLastTotalRx(sinkA->GetTotalRx());
    sinkB->SetLastTotalRx(sinkB->GetTotalRx());
    
    // Write to file
    os << timeNow << "," << throughputA << "," << throughputB << std::endl;
    
    // Schedule next call
    Simulator::Schedule(Seconds(interval), &MonitorThroughput, sinkA, sinkB, std::ref(os), interval);
}

int main(int argc, char *argv[]) {
    // Enable logging
    LogComponentEnable("WifiHandoverSimulation", LOG_LEVEL_INFO);
    
    // Configurable simulation parameters
    double simTime = 3.0;         // Total simulation time (5 minutes)
    double firstMigration = 1.0;  // First migration time (at 100 seconds)
    double secondMigration = 2.0; // Second migration time (at 200 seconds)
    double throughputInterval = 1.0; // Interval for throughput measurement
    
    // Command line arguments
    CommandLine cmd;
    cmd.AddValue("simTime", "Total simulation time in seconds", simTime);
    cmd.AddValue("firstMigration", "Time of first user migration in seconds", firstMigration);
    cmd.AddValue("secondMigration", "Time of second user migration in seconds", secondMigration);
    cmd.Parse(argc, argv);
    
    // Print simulation parameters
    NS_LOG_INFO("Simulation parameters:");
    NS_LOG_INFO("- Total simulation time: " << simTime << " seconds");
    NS_LOG_INFO("- First migration time: " << firstMigration << " seconds");
    NS_LOG_INFO("- Second migration time: " << secondMigration << " seconds");
    
    uint32_t totalStations = 32;  // 16 stations per AP
    
    NS_LOG_INFO("Creating topology");
    
    // Create nodes: 2 APs and 32 stations (16 per AP)
    NodeContainer apNodes;
    apNodes.Create(2);
    
    NodeContainer staNodes;
    staNodes.Create(totalStations);
    
    // Create wifi devices
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);
    
    // Set up PHY
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    
    // Configure AP
    WifiMacHelper mac;
    Ssid ssid = Ssid("wifi-handover-network");
    
    // AP configuration
    mac.SetType("ns3::ApWifiMac",
               "Ssid", SsidValue(ssid),
               "BeaconGeneration", BooleanValue(true),
               "BeaconInterval", TimeValue(MicroSeconds(102400)));
    
    // Install on AP nodes
    NetDeviceContainer apDevices = wifi.Install(phy, mac, apNodes);
    
    // STA configuration
    mac.SetType("ns3::StaWifiMac",
               "Ssid", SsidValue(ssid),
               "ActiveProbing", BooleanValue(false));
    
    // Install on STA nodes
    NetDeviceContainer staDevices = wifi.Install(phy, mac, staNodes);
    
    // Set mobility model
    MobilityHelper mobility;
    
    // Position APs
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0.0, 0.0, 0.0));  // AP A position
    positionAlloc->Add(Vector(50.0, 0.0, 0.0)); // AP B position
    
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(apNodes);
    
    // Random walk for stations
    mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                 "X", DoubleValue(25.0),
                                 "Y", DoubleValue(0.0),
                                 "Rho", StringValue("ns3::UniformRandomVariable[Min=0|Max=30]"));
    
    mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                             "Bounds", RectangleValue(Rectangle(-50, 100, -50, 50)),
                             "Speed", StringValue("ns3::ConstantRandomVariable[Constant=2.0]"));
    mobility.Install(staNodes);
    
    // Install Internet stack
    InternetStackHelper stack;
    stack.Install(apNodes);
    stack.Install(staNodes);
    
    // Assign IP addresses
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    
    Ipv4InterfaceContainer apInterfaces = address.Assign(apDevices);
    Ipv4InterfaceContainer staInterfaces = address.Assign(staDevices);
    
    // Setup servers on APs
    uint16_t port = 9;
    
    // Create packet sinks on APs using our custom sink
    ObjectFactory factory;
    factory.SetTypeId("ns3::ThroughputSink");
    factory.Set("Protocol", StringValue("ns3::UdpSocketFactory"));
    factory.Set("Local", AddressValue(InetSocketAddress(Ipv4Address::GetAny(), port)));

    Ptr<ThroughputSink> sinkA = DynamicCast<ThroughputSink>(factory.Create<ThroughputSink>());
    Ptr<ThroughputSink> sinkB = DynamicCast<ThroughputSink>(factory.Create<ThroughputSink>());
    
    apNodes.Get(0)->AddApplication(sinkA);
    apNodes.Get(1)->AddApplication(sinkB);
    
    sinkA->SetStartTime(Seconds(0.0));
    sinkA->SetStopTime(Seconds(simTime));
    sinkB->SetStartTime(Seconds(0.0));
    sinkB->SetStopTime(Seconds(simTime));
    
    // Initialize the counters
    sinkA->SetLastTotalRx(0);
    sinkB->SetLastTotalRx(0);
    
    // Full queue traffic generation (OnOff Application)
    OnOffHelper onoff("ns3::UdpSocketFactory", Address());
    onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    onoff.SetAttribute("DataRate", DataRateValue(DataRate("1Mbps")));
    onoff.SetAttribute("PacketSize", UintegerValue(1024));
    
    // Install applications on all stations
    ApplicationContainer clientApps;
    
    // First 16 stations send to AP A
    for (uint32_t i = 0; i < 16; i++) {
        AddressValue remoteAddress(InetSocketAddress(apInterfaces.GetAddress(0), port));
        onoff.SetAttribute("Remote", remoteAddress);
        clientApps.Add(onoff.Install(staNodes.Get(i)));
    }
    
    // Next 16 stations send to AP B
    for (uint32_t i = 16; i < 32; i++) {
        AddressValue remoteAddress(InetSocketAddress(apInterfaces.GetAddress(1), port));
        onoff.SetAttribute("Remote", remoteAddress);
        clientApps.Add(onoff.Install(staNodes.Get(i)));
    }
    
    clientApps.Start(Seconds(1.0));
    clientApps.Stop(Seconds(simTime - 1));
    
    // Create user migration handler
    UserMigrationHandler migrationHandler(apNodes.Get(0), apNodes.Get(1), staNodes, staDevices);
    
    // Initialize output files
    std::ofstream throughputFile("throughput.csv");
    throughputFile << "Time,ThroughputA_Mbps,ThroughputB_Mbps" << std::endl;
    
    std::ofstream userDistFile("user_distribution.csv");
    userDistFile << "Time,UsersAtA,UsersAtB" << std::endl;
    userDistFile << "0,16,16" << std::endl;
    
    // Schedule user migrations using the configurable parameters
    migrationHandler.ScheduleMigration(firstMigration, 0.25, 0.50);  // 1st migration: 25% A→B, 50% B→A
    migrationHandler.ScheduleMigration(secondMigration, 0.50, 0.50); // 2nd migration: 50% A→B, 50% B→A
    
    // Schedule throughput monitoring
    Simulator::Schedule(Seconds(1.0), &MonitorThroughput, 
                      sinkA, sinkB, std::ref(throughputFile), throughputInterval);
    
    // Run simulation
    NS_LOG_INFO("Starting simulation for " << simTime << " seconds");
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    
    // Print final statistics
    NS_LOG_INFO("Simulation complete");
    NS_LOG_INFO("Total bytes received at AP A: " << sinkA->GetTotalRx());
    NS_LOG_INFO("Total bytes received at AP B: " << sinkB->GetTotalRx());
    
    Simulator::Destroy();
    return 0;
}
