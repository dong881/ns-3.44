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
#include <vector>

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

// Improved centralized handover controller
class HandoverController {
public:
    // Handover plan structure
    struct HandoverPlan {
        double time;           // When to execute handover
        uint32_t moveFromAToB; // Number of users to move from A to B
        uint32_t moveFromBToA; // Number of users to move from B to A
    };

    HandoverController(NodeContainer apNodes, NodeContainer staNodes, NetDeviceContainer staDevices) 
        : m_apNodes(apNodes), m_staNodes(staNodes), m_staDevices(staDevices) {
        
        // Initialize AP-STA associations (first 16 to AP A, rest to AP B)
        for (uint32_t i = 0; i < staNodes.GetN(); i++) {
            if (i < 16) {
                m_staToAp[i] = 0; // Associated with AP A
            } else {
                m_staToAp[i] = 1; // Associated with AP B
            }
        }
        
        // Get MAC addresses for all APs
        for (uint32_t i = 0; i < m_apNodes.GetN(); i++) {
            Ptr<WifiNetDevice> apDev = DynamicCast<WifiNetDevice>(m_apNodes.Get(i)->GetDevice(0));
            if (apDev) {
                m_apMacs.push_back(Mac48Address::ConvertFrom(apDev->GetAddress()));
                NS_LOG_INFO("AP " << i << " MAC address: " << m_apMacs[i]);
            }
        }
    }
    
    // Add a handover plan to the schedule
    void AddHandoverPlan(double time, uint32_t moveFromAToB, uint32_t moveFromBToA) {
        HandoverPlan plan;
        plan.time = time;
        plan.moveFromAToB = moveFromAToB;
        plan.moveFromBToA = moveFromBToA;
        
        m_handoverPlans.push_back(plan);
        
        // Schedule this handover
        Simulator::Schedule(Seconds(time), &HandoverController::ExecuteHandover, this, moveFromAToB, moveFromBToA);
        
        NS_LOG_INFO("Scheduled handover at " << time << "s: " << moveFromAToB << " users A->B, " 
                    << moveFromBToA << " users B->A");
    }
    
    // Get the current distribution of users
    std::pair<uint32_t, uint32_t> GetUserDistribution() const {
        uint32_t countA = 0;
        uint32_t countB = 0;
        
        for (auto const& pair : m_staToAp) {
            if (pair.second == 0) countA++;
            else if (pair.second == 1) countB++;
        }
        
        return std::make_pair(countA, countB);
    }
    
    // Log the user distribution to file
    void LogUserDistribution(std::ofstream& os) {
        auto distribution = GetUserDistribution();
        double time = Simulator::Now().GetSeconds();
        
        os << time << "," << distribution.first << "," << distribution.second << std::endl;
        
        NS_LOG_INFO("Time " << time << "s: " << distribution.first << " users at AP A, " 
                   << distribution.second << " users at AP B");
    }

private:
    void ExecuteHandover(uint32_t moveFromAToB, uint32_t moveFromBToA) {
        NS_LOG_INFO("Executing handover: " << moveFromAToB << " users A->B, " << moveFromBToA << " users B->A");
        
        // Get current lists of users at each AP
        std::vector<uint32_t> usersAtA;
        std::vector<uint32_t> usersAtB;
        
        for (auto const& pair : m_staToAp) {
            if (pair.second == 0) {
                usersAtA.push_back(pair.first);
            } else if (pair.second == 1) {
                usersAtB.push_back(pair.first);
            }
        }
        
        // Check if we have enough users to move
        moveFromAToB = std::min(moveFromAToB, static_cast<uint32_t>(usersAtA.size()));
        moveFromBToA = std::min(moveFromBToA, static_cast<uint32_t>(usersAtB.size()));
        
        // Execute A to B handover
        for (uint32_t i = 0; i < moveFromAToB; i++) {
            uint32_t staIndex = usersAtA[i];
            Ptr<WifiNetDevice> staDev = DynamicCast<WifiNetDevice>(m_staDevices.Get(staIndex));
            
            if (staDev && staDev->GetMac()) {
                staDev->GetMac()->SetBssid(m_apMacs[1], 0); // Move to AP B (index 1)
                m_staToAp[staIndex] = 1;
                NS_LOG_INFO("Station " << staIndex << " moved from AP A to AP B");
            }
        }
        
        // Execute B to A handover
        for (uint32_t i = 0; i < moveFromBToA; i++) {
            uint32_t staIndex = usersAtB[i];
            Ptr<WifiNetDevice> staDev = DynamicCast<WifiNetDevice>(m_staDevices.Get(staIndex));
            
            if (staDev && staDev->GetMac()) {
                staDev->GetMac()->SetBssid(m_apMacs[0], 0); // Move to AP A (index 0)
                m_staToAp[staIndex] = 0;
                NS_LOG_INFO("Station " << staIndex << " moved from AP B to AP A");
            }
        }
        
        // Log user distribution after handover
        std::ofstream userDist("user_distribution.csv", std::ios_base::app);
        LogUserDistribution(userDist);
    }

    NodeContainer m_apNodes;
    NodeContainer m_staNodes;
    NetDeviceContainer m_staDevices;
    std::map<uint32_t, int> m_staToAp;      // Maps station index to AP index
    std::vector<Mac48Address> m_apMacs;      // MAC addresses of APs
    std::vector<HandoverPlan> m_handoverPlans; // List of handover plans
};

// Generic function to monitor throughput of any AP
void MonitorApThroughput(std::vector<Ptr<ThroughputSink>> sinks, std::ofstream& os, double interval) {
    double timeNow = Simulator::Now().GetSeconds();
    
    // Write time
    os << timeNow;
    
    // Calculate and record throughput for each AP
    for (uint32_t i = 0; i < sinks.size(); i++) {
        double throughput = (sinks[i]->GetTotalRx() - sinks[i]->GetLastTotalRx()) * 8.0 / interval / 1000000;
        sinks[i]->SetLastTotalRx(sinks[i]->GetTotalRx());
        os << "," << throughput;
    }
    os << std::endl;
    
    // Schedule next call
    Simulator::Schedule(Seconds(interval), &MonitorApThroughput, sinks, std::ref(os), interval);
}

int main(int argc, char *argv[]) {
    // Enable logging
    LogComponentEnable("WifiHandoverSimulation", LOG_LEVEL_INFO);
    
    // Configurable simulation parameters
    double simTime = 3.0;         // Total simulation time (5 minutes)
    double firstMigration = 1.0;  // First migration time (at 100 seconds)
    double secondMigration = 2.0; // Second migration time (at 200 seconds)
    uint32_t firstMoveAtoB = 4;     // Number of users to move A->B in first migration (25% of 16)
    uint32_t firstMoveBtoA = 8;     // Number of users to move B->A in first migration (50% of 16)
    uint32_t secondMoveAtoB = 10;    // Number of users to move A->B in second migration (50% of 20)
    uint32_t secondMoveBtoA = 6;    // Number of users to move B->A in second migration (50% of 12)
    double throughputInterval = 1.0; // Interval for throughput measurement
    
    // Command line arguments
    CommandLine cmd;
    cmd.AddValue("simTime", "Total simulation time in seconds", simTime);
    cmd.AddValue("firstMigration", "Time of first user migration in seconds", firstMigration);
    cmd.AddValue("secondMigration", "Time of second user migration in seconds", secondMigration);
    cmd.AddValue("firstMoveAtoB", "Users to move A->B in first migration", firstMoveAtoB);
    cmd.AddValue("firstMoveBtoA", "Users to move B->A in first migration", firstMoveBtoA);
    cmd.AddValue("secondMoveAtoB", "Users to move A->B in second migration", secondMoveAtoB);
    cmd.AddValue("secondMoveBtoA", "Users to move B->A in second migration", secondMoveBtoA);
    cmd.Parse(argc, argv);
    
    // Print simulation parameters
    NS_LOG_INFO("Simulation parameters:");
    NS_LOG_INFO("- Total simulation time: " << simTime << " seconds");
    NS_LOG_INFO("- First migration at " << firstMigration << "s: " 
                << firstMoveAtoB << " users A->B, " << firstMoveBtoA << " users B->A");
    NS_LOG_INFO("- Second migration at " << secondMigration << "s: " 
                << secondMoveAtoB << " users A->B, " << secondMoveBtoA << " users B->A");
    
    uint32_t nAps = 2;
    uint32_t totalStations = 32;  // 16 stations per AP
    
    NS_LOG_INFO("Creating topology with " << nAps << " APs and " << totalStations << " stations");
    
    // Create nodes: 2 APs and 32 stations (16 per AP initially)
    NodeContainer apNodes;
    apNodes.Create(nAps);
    
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
    
    // AP configuration - all APs use the same code/configuration
    mac.SetType("ns3::ApWifiMac",
               "Ssid", SsidValue(ssid),
               "BeaconGeneration", BooleanValue(true),
               "BeaconInterval", TimeValue(MicroSeconds(102400)));
    
    // Install on AP nodes
    NetDeviceContainer apDevices = wifi.Install(phy, mac, apNodes);
    
    // STA configuration - all stations use the same code/configuration
    mac.SetType("ns3::StaWifiMac",
               "Ssid", SsidValue(ssid),
               "ActiveProbing", BooleanValue(false));
    
    // Install on STA nodes
    NetDeviceContainer staDevices = wifi.Install(phy, mac, staNodes);
    
    // Set mobility model
    MobilityHelper mobility;
    
    // Position APs
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < nAps; i++) {
        // Position APs at different locations (50 units apart)
        positionAlloc->Add(Vector(i * 50.0, 0.0, 0.0));
    }
    
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(apNodes);
    
    // Random walk for stations
    mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                 "X", DoubleValue(25.0), // Centered between the APs
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
    
    // Create packet sinks on all APs using our custom sink
    std::vector<Ptr<ThroughputSink>> sinks;
    
    for (uint32_t i = 0; i < nAps; i++) {
        ObjectFactory factory;
        factory.SetTypeId("ns3::ThroughputSink");
        factory.Set("Protocol", StringValue("ns3::UdpSocketFactory"));
        factory.Set("Local", AddressValue(InetSocketAddress(Ipv4Address::GetAny(), port)));

        Ptr<ThroughputSink> sink = DynamicCast<ThroughputSink>(factory.Create<ThroughputSink>());
        apNodes.Get(i)->AddApplication(sink);
        sink->SetStartTime(Seconds(0.0));
        sink->SetStopTime(Seconds(simTime));
        sink->SetLastTotalRx(0);
        sinks.push_back(sink);
    }
    
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
    
    // Create handover controller
    HandoverController handoverController(apNodes, staNodes, staDevices);
    
    // Initialize output files
    std::ofstream throughputFile("throughput.csv");
    throughputFile << "Time,ThroughputA_Mbps,ThroughputB_Mbps" << std::endl;
    
    std::ofstream userDistFile("user_distribution.csv");
    userDistFile << "Time,UsersAtA,UsersAtB" << std::endl;
    userDistFile << "0,16,16" << std::endl;
    
    // Schedule handovers with specific numbers of users to move
    handoverController.AddHandoverPlan(firstMigration, firstMoveAtoB, firstMoveBtoA);
    handoverController.AddHandoverPlan(secondMigration, secondMoveAtoB, secondMoveBtoA);
    
    // Schedule throughput monitoring
    Simulator::Schedule(Seconds(1.0), &MonitorApThroughput, sinks, std::ref(throughputFile), throughputInterval);
    
    // Run simulation
    NS_LOG_INFO("Starting simulation for " << simTime << " seconds");
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    
    // Print final statistics
    NS_LOG_INFO("Simulation complete");
    for (uint32_t i = 0; i < nAps; i++) {
        NS_LOG_INFO("Total bytes received at AP " << i << ": " << sinks[i]->GetTotalRx());
    }
    
    Simulator::Destroy();
    return 0;
}
