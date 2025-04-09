#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"
#include <fstream>
#include <vector>
#include <iomanip>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiHandoverSimulation");

// Global variables for statistics
std::map<uint32_t, std::pair<double, uint32_t>> apAStats;  // throughput, connected users
std::map<uint32_t, std::pair<double, uint32_t>> apBStats;

void MoveNode(Ptr<Node> node, const Vector& targetPosition) {
    Ptr<ConstantVelocityMobilityModel> mobility = 
        node->GetObject<ConstantVelocityMobilityModel>();
    
    // Get current position
    Vector currentPos = mobility->GetPosition();
    
    // Calculate direction vector
    Vector direction(targetPosition.x - currentPos.x,
                    targetPosition.y - currentPos.y,
                    0);
    
    // Normalize and set velocity
    double distance = std::sqrt(std::pow(direction.x, 2) + std::pow(direction.y, 2));
    if (distance > 0) {
        mobility->SetVelocity(Vector(direction.x / distance * 5.0,
                                   direction.y / distance * 5.0,
                                   0));
    }
}

void
UpdateUserCounts(NodeContainer& nodesA, NodeContainer& nodesB, 
                Ptr<Node> apA, Ptr<Node> apB, uint32_t timeStep)
{
    uint32_t usersA = 0;
    uint32_t usersB = 0;
    
    // Count users based on distance to APs
    for (uint32_t i = 0; i < nodesA.GetN(); ++i) {
        Ptr<MobilityModel> mobModel = nodesA.Get(i)->GetObject<MobilityModel>();
        Vector pos = mobModel->GetPosition();
        
        double distToA = std::sqrt(std::pow(pos.x + 50, 2) + std::pow(pos.y, 2));
        double distToB = std::sqrt(std::pow(pos.x - 50, 2) + std::pow(pos.y, 2));
        
        if (distToA < distToB) usersA++;
        else usersB++;
    }
    
    for (uint32_t i = 0; i < nodesB.GetN(); ++i) {
        Ptr<MobilityModel> mobModel = nodesB.Get(i)->GetObject<MobilityModel>();
        Vector pos = mobModel->GetPosition();
        
        double distToA = std::sqrt(std::pow(pos.x + 50, 2) + std::pow(pos.y, 2));
        double distToB = std::sqrt(std::pow(pos.x - 50, 2) + std::pow(pos.y, 2));
        
        if (distToA < distToB) usersA++;
        else usersB++;
    }
    
    // Update statistics
    if (apAStats.find(timeStep) != apAStats.end()) {
        apAStats[timeStep].second = usersA;
    }
    if (apBStats.find(timeStep) != apBStats.end()) {
        apBStats[timeStep].second = usersB;
    }
    
    // Log user distribution
    std::ofstream outFile("/home/ming/multimedia-wireless-networks/a2/src/user_distribution.csv", std::ios::app);
    outFile << timeStep << "," << usersA << "," << usersB << std::endl;
    outFile.close();
}

void
ThroughputMonitor(FlowMonitorHelper* fmhelper, Ptr<FlowMonitor> flowMon, 
                  NodeContainer& nodesA, NodeContainer& nodesB,
                  Ptr<Node> apA, Ptr<Node> apB, double em)
{
    std::map<FlowId, FlowMonitor::FlowStats> flowStats = flowMon->GetFlowStats();
    double throughputA = 0.0;
    double throughputB = 0.0;
    uint32_t timeStep = static_cast<uint32_t>(em);
    
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = flowStats.begin(); i != flowStats.end(); ++i) {
        double localThroughput = i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds()) / 1024 / 1024;
        
        Ipv4FlowClassifier::FiveTuple t = StaticCast<Ipv4FlowClassifier>(fmhelper->GetClassifier())->FindFlow(i->first);
        if (t.destinationAddress == "10.1.1.1") { // AP A
            throughputA += localThroughput;
        } else if (t.destinationAddress == "10.1.2.1") { // AP B
            throughputB += localThroughput;
        }
    }
    
    // Store statistics
    apAStats[timeStep] = std::make_pair(throughputA, 0);
    apBStats[timeStep] = std::make_pair(throughputB, 0);
    
    // Update user counts
    UpdateUserCounts(nodesA, nodesB, apA, apB, timeStep);
    
    // Log detailed statistics
    std::ofstream outFile("/home/ming/multimedia-wireless-networks/a2/src/detailed_stats.csv", std::ios::app);
    outFile << std::fixed << std::setprecision(6)
            << timeStep << ","
            << throughputA << ","
            << throughputB << ","
            << apAStats[timeStep].second << ","
            << apBStats[timeStep].second << std::endl;
    outFile.close();
    
    Simulator::Schedule(Seconds(1.0), &ThroughputMonitor, fmhelper, flowMon, 
                       std::ref(nodesA), std::ref(nodesB), apA, apB, em + 1);
}

int main(int argc, char* argv[])
{
    // Enable logging
    LogComponentEnable("WifiHandoverSimulation", LOG_LEVEL_INFO);
    
    // Clean previous statistics files
    std::ofstream cleanFile1("/home/ming/multimedia-wireless-networks/a2/src/detailed_stats.csv");
    cleanFile1 << "Time,ThroughputA,ThroughputB,UsersA,UsersB" << std::endl;
    cleanFile1.close();
    
    std::ofstream cleanFile2("/home/ming/multimedia-wireless-networks/a2/src/user_distribution.csv");
    cleanFile2 << "Time,UsersA,UsersB" << std::endl;
    cleanFile2.close();
    
    // Create node containers
    NodeContainer wifiStaNodesA;
    wifiStaNodesA.Create(16);
    NodeContainer wifiStaNodesB;
    wifiStaNodesB.Create(16);
    NodeContainer wifiApNodes;
    wifiApNodes.Create(2);
    
    // Create wifi helper with improved settings
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);
    wifi.SetRemoteStationManager("ns3::IdealWifiManager");
    
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    
    WifiMacHelper mac;
    Ssid ssid1 = Ssid("AP-A");
    Ssid ssid2 = Ssid("AP-B");
    
    // Configure APs with enhanced settings
    mac.SetType("ns3::ApWifiMac",
                "Ssid", SsidValue(ssid1),
                "EnableBeaconJitter", BooleanValue(false),
                "BeaconInterval", TimeValue(MicroSeconds(102400)));
    NetDeviceContainer apDevicesA = wifi.Install(phy, mac, wifiApNodes.Get(0));
    
    mac.SetType("ns3::ApWifiMac",
                "Ssid", SsidValue(ssid2),
                "EnableBeaconJitter", BooleanValue(false),
                "BeaconInterval", TimeValue(MicroSeconds(102400)));
    NetDeviceContainer apDevicesB = wifi.Install(phy, mac, wifiApNodes.Get(1));
    
    // Configure STAs with roaming support
    mac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssid1),
                "ActiveProbing", BooleanValue(true));
    NetDeviceContainer staDevicesA = wifi.Install(phy, mac, wifiStaNodesA);
    
    mac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssid2),
                "ActiveProbing", BooleanValue(true));
    NetDeviceContainer staDevicesB = wifi.Install(phy, mac, wifiStaNodesB);
    
    // Mobility model
    MobilityHelper mobility;
    
    // Position of APs
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(-50.0, 0.0, 0.0));  // AP A
    positionAlloc->Add(Vector(50.0, 0.0, 0.0));   // AP B
    
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(wifiApNodes);
    
    // Initial positions for STAs with controlled mobility
    mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                "X", DoubleValue(-50.0),
                                "Y", DoubleValue(0.0),
                                "Rho", StringValue("ns3::UniformRandomVariable[Min=0|Max=10]"));
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(wifiStaNodesA);
    
    mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                "X", DoubleValue(50.0),
                                "Y", DoubleValue(0.0),
                                "Rho", StringValue("ns3::UniformRandomVariable[Min=0|Max=10]"));
    mobility.Install(wifiStaNodesB);
    
    // Internet stack
    InternetStackHelper stack;
    stack.Install(wifiApNodes);
    stack.Install(wifiStaNodesA);
    stack.Install(wifiStaNodesB);
    
    // Assign IP addresses
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterfaceA = address.Assign(apDevicesA);
    Ipv4InterfaceContainer staInterfacesA = address.Assign(staDevicesA);
    
    address.SetBase("10.1.2.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterfaceB = address.Assign(apDevicesB);
    Ipv4InterfaceContainer staInterfacesB = address.Assign(staDevicesB);
    
    // Traffic generation
    uint16_t port = 9;
    ApplicationContainer serverApps;
    
    UdpServerHelper serverA(port);
    serverApps.Add(serverA.Install(wifiApNodes.Get(0)));
    UdpServerHelper serverB(port);
    serverApps.Add(serverB.Install(wifiApNodes.Get(1)));
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(20.0));  // Changed to 20s total simulation time
    
    ApplicationContainer clientApps;
    
    // Configure UDP clients with higher data rate
    UdpClientHelper clientA(apInterfaceA.GetAddress(0), port);
    clientA.SetAttribute("MaxPackets", UintegerValue(4294967295u));
    clientA.SetAttribute("Interval", TimeValue(Seconds(0.01)));
    clientA.SetAttribute("PacketSize", UintegerValue(1400));
    
    UdpClientHelper clientB(apInterfaceB.GetAddress(0), port);
    clientB.SetAttribute("MaxPackets", UintegerValue(4294967295u));
    clientB.SetAttribute("Interval", TimeValue(Seconds(0.01)));
    clientB.SetAttribute("PacketSize", UintegerValue(1400));
    
    // Install clients
    for (uint32_t i = 0; i < wifiStaNodesA.GetN(); ++i) {
        clientApps.Add(clientA.Install(wifiStaNodesA.Get(i)));
    }
    for (uint32_t i = 0; i < wifiStaNodesB.GetN(); ++i) {
        clientApps.Add(clientB.Install(wifiStaNodesB.Get(i)));
    }
    
    clientApps.Start(Seconds(2.0));
    clientApps.Stop(Seconds(20.0));  // Changed to 20s total simulation time
    
    // Flow monitor
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();
    
    // Schedule throughput monitoring
    Simulator::Schedule(Seconds(1.0), &ThroughputMonitor, &flowmon, monitor,
                       std::ref(wifiStaNodesA), std::ref(wifiStaNodesB),
                       wifiApNodes.Get(0), wifiApNodes.Get(1), 1.0);
    
    // Schedule user movements with precise timing and improved movement
    Simulator::Schedule(Seconds(10.0), [&wifiStaNodesA, &wifiStaNodesB]() {
        NS_LOG_INFO("Executing first movement at t=10s");
        // Move 25% from A to B (4 users)
        for (uint32_t i = 0; i < 4; ++i) {
            Vector targetPos(50.0 + (rand() % 5), (rand() % 10) - 5, 0);
            MoveNode(wifiStaNodesA.Get(i), targetPos);
            // Stop previous nodes
            Ptr<ConstantVelocityMobilityModel> mobility = 
                wifiStaNodesA.Get(i)->GetObject<ConstantVelocityMobilityModel>();
            Simulator::Schedule(Seconds(2.0), &ConstantVelocityMobilityModel::SetVelocity,
                              mobility, Vector(0, 0, 0));
        }
        // Move 50% from B to A (8 users)
        for (uint32_t i = 0; i < 8; ++i) {
            Vector targetPos(-50.0 + (rand() % 5), (rand() % 10) - 5, 0);
            MoveNode(wifiStaNodesB.Get(i), targetPos);
            // Stop nodes after movement
            Ptr<ConstantVelocityMobilityModel> mobility = 
                wifiStaNodesB.Get(i)->GetObject<ConstantVelocityMobilityModel>();
            Simulator::Schedule(Seconds(2.0), &ConstantVelocityMobilityModel::SetVelocity,
                              mobility, Vector(0, 0, 0));
        }
    });
    
    Simulator::Schedule(Seconds(15.0), [&wifiStaNodesA, &wifiStaNodesB]() {
        NS_LOG_INFO("Executing second movement at t=15s");
        // Move 50% from A to B (10 users)
        for (uint32_t i = 0; i < 10; ++i) {
            Vector targetPos(50.0 + (rand() % 5), (rand() % 10) - 5, 0);
            MoveNode(wifiStaNodesA.Get(i), targetPos);
            // Stop nodes after movement
            Ptr<ConstantVelocityMobilityModel> mobility = 
                wifiStaNodesA.Get(i)->GetObject<ConstantVelocityMobilityModel>();
            Simulator::Schedule(Seconds(2.0), &ConstantVelocityMobilityModel::SetVelocity,
                              mobility, Vector(0, 0, 0));
        }
        // Move 50% from B to A (6 users)
        for (uint32_t i = 0; i < 6; ++i) {
            Vector targetPos(-50.0 + (rand() % 5), (rand() % 10) - 5, 0);
            MoveNode(wifiStaNodesB.Get(i), targetPos);
            // Stop nodes after movement
            Ptr<ConstantVelocityMobilityModel> mobility = 
                wifiStaNodesB.Get(i)->GetObject<ConstantVelocityMobilityModel>();
            Simulator::Schedule(Seconds(2.0), &ConstantVelocityMobilityModel::SetVelocity,
                              mobility, Vector(0, 0, 0));
        }
    });
    
    // Animation
    AnimationInterface anim("wifi-handover-simulation.xml");
    
    // Set node colors and descriptions
    for (uint32_t i = 0; i < wifiApNodes.GetN(); ++i) {
        anim.UpdateNodeDescription(wifiApNodes.Get(i), "AP");
        anim.UpdateNodeColor(wifiApNodes.Get(i), 255, 0, 0); // Red for APs
    }
    
    for (uint32_t i = 0; i < wifiStaNodesA.GetN(); ++i) {
        anim.UpdateNodeDescription(wifiStaNodesA.Get(i), "STA-A");
        anim.UpdateNodeColor(wifiStaNodesA.Get(i), 0, 255, 0); // Green for group A
    }
    
    for (uint32_t i = 0; i < wifiStaNodesB.GetN(); ++i) {
        anim.UpdateNodeDescription(wifiStaNodesB.Get(i), "STA-B");
        anim.UpdateNodeColor(wifiStaNodesB.Get(i), 0, 0, 255); // Blue for group B
    }
    
    anim.EnablePacketMetadata(true);
    
    Simulator::Stop(Seconds(20.0));  // Changed to 20s total simulation time
    Simulator::Run();
    Simulator::Destroy();
    
    return 0;
}