#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/ipv4-flow-classifier.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiFullQueueSimulation");

// 自訂移動事件處理類別
class UserMigrationHandler {
public:
    UserMigrationHandler(
        NodeContainer& apNodes,
        NodeContainer& staNodes,
        NetDeviceContainer& apDevices,
        NetDeviceContainer& staDevices,
        uint32_t nStaPerAp)
        : m_apNodes(apNodes),
          m_staNodes(staNodes),
          m_apDevices(apDevices),
          m_staDevices(staDevices),
          nStaPerAp(nStaPerAp) {}

    void MigrateUsers(double time, double percentAtoB, double percentBtoA) {
        Simulator::Schedule(Seconds(time), &UserMigrationHandler::DoMigrate, this, percentAtoB, percentBtoA);
    }

private:
    void DoMigrate(double percentAtoB, double percentBtoA) {
        // 獲取當前時間戳記
        double currentTime = Simulator::Now().GetSeconds();
        
        // 記錄日誌
        NS_LOG_INFO("Executing user migration at " << currentTime << "s");

        // 轉換 AP 地址為 Mac48Address 類型
        Mac48Address ap1Addr = Mac48Address::ConvertFrom(m_apDevices.Get(0)->GetAddress());
        Mac48Address ap2Addr = Mac48Address::ConvertFrom(m_apDevices.Get(1)->GetAddress());

        // 計算遷移數量
        uint32_t moveAtoB = static_cast<uint32_t>(nStaPerAp * percentAtoB);
        uint32_t moveBtoA = static_cast<uint32_t>(nStaPerAp * percentBtoA);
        
        NS_LOG_INFO("Moving " << moveAtoB << " stations from AP1 to AP2");
        NS_LOG_INFO("Moving " << moveBtoA << " stations from AP2 to AP1");

        // 執行遷移邏輯（假設使用單一鏈路，linkId=0）
        for (uint32_t i = 0; i < moveAtoB; ++i) {
            Ptr<WifiNetDevice> staDev = DynamicCast<WifiNetDevice>(m_staDevices.Get(i));
            if (!staDev) {
                NS_LOG_ERROR("Failed to cast device " << i << " to WifiNetDevice");
                continue;
            }
            
            Ptr<WifiMac> mac = staDev->GetMac();
            if (!mac) {
                NS_LOG_ERROR("Failed to get MAC for device " << i);
                continue;
            }
            
            mac->SetBssid(ap2Addr, 0);
        }

        for (uint32_t j = nStaPerAp; j < nStaPerAp + moveBtoA; ++j) {
            if (j >= m_staDevices.GetN()) {
                NS_LOG_ERROR("Device index " << j << " out of bounds");
                continue;
            }
            
            Ptr<WifiNetDevice> staDev = DynamicCast<WifiNetDevice>(m_staDevices.Get(j));
            if (!staDev) {
                NS_LOG_ERROR("Failed to cast device " << j << " to WifiNetDevice");
                continue;
            }
            
            Ptr<WifiMac> mac = staDev->GetMac();
            if (!mac) {
                NS_LOG_ERROR("Failed to get MAC for device " << j);
                continue;
            }
            
            mac->SetBssid(ap1Addr, 0);
        }

        // 記錄用戶分佈
        std::ofstream userFile("user_distribution.csv", std::ios_base::app);
        uint32_t countA = 0, countB = 0;
        for (uint32_t k = 0; k < m_staNodes.GetN(); ++k) {
            Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice>(m_staDevices.Get(k));
            if (!dev || !dev->GetMac()) {
                NS_LOG_ERROR("Invalid device or MAC at index " << k);
                continue;
            }
            
            if (dev->GetMac()->GetBssid(0) == ap1Addr) countA++;
            else countB++;
        }
        userFile << currentTime << "," << countA << "," << countB << "\n";
        NS_LOG_INFO("User distribution: AP1=" << countA << ", AP2=" << countB);
    }

    NodeContainer& m_apNodes;
    NodeContainer& m_staNodes;
    NetDeviceContainer& m_apDevices;
    NetDeviceContainer& m_staDevices;
    uint32_t nStaPerAp;
};

// 簡化的吞吐量監控函數
void ThroughputMonitor(Ptr<FlowMonitor> monitor, std::ofstream& outFile) {
    double totalTime = Simulator::Now().GetSeconds();
    if (totalTime == 0.0) {
        // 避免除以零錯誤
        Simulator::Schedule(Seconds(0.5), &ThroughputMonitor, monitor, std::ref(outFile));
        return;
    }

    // 獲取流量統計
    double throughputTotal = 0;
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
    
    // 計算總吞吐量
    for (auto& stat : stats) {
        throughputTotal += stat.second.rxBytes * 8.0 / (totalTime * 1000000); // Convert to Mbps
    }

    // 寫入檔案
    outFile << totalTime << "," << throughputTotal << "\n";

    // 每0.5秒記錄一次
    Simulator::Schedule(Seconds(0.5), &ThroughputMonitor, monitor, std::ref(outFile));
}

int main(int argc, char *argv[]) {
    // Enable logging for debugging
    LogComponentEnable("WifiFullQueueSimulation", LOG_LEVEL_INFO);
    
    // 基礎參數設定
    uint32_t nAp = 2;
    uint32_t nStaPerAp = 16;
    double totalTime = 15.0; // 總模擬時間為 15 秒

    // 建立節點
    NodeContainer apNodes;
    apNodes.Create(nAp);

    NodeContainer staNodes;
    staNodes.Create(nAp * nStaPerAp);

    // WiFi 設定
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);

    YansWifiPhyHelper phy;
    phy.Set("ChannelSettings", StringValue("{0, 40, BAND_5GHZ, 0}"));

    WifiMacHelper mac;
    Ssid ssid = Ssid("ns3-wifi-network");

    // 設定AP
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    NetDeviceContainer apDevices = wifi.Install(phy, mac, apNodes);

    // 設定STA
    mac.SetType("ns3::StaWifiMac", 
               "Ssid", SsidValue(ssid),
               "ActiveProbing", BooleanValue(false));
    NetDeviceContainer staDevices = wifi.Install(phy, mac, staNodes);
    
    // Verify the devices were created properly
    NS_LOG_INFO("Created " << apDevices.GetN() << " AP devices");
    NS_LOG_INFO("Created " << staDevices.GetN() << " STA devices");
    
    for (uint32_t i = 0; i < apDevices.GetN(); i++) {
        Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice>(apDevices.Get(i));
        if (!dev || !dev->GetMac()) {
            NS_LOG_ERROR("AP device " << i << " is invalid");
        } else {
            NS_LOG_INFO("AP " << i << " has valid MAC");
        }
    }
    
    for (uint32_t i = 0; i < staDevices.GetN(); i++) {
        Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice>(staDevices.Get(i));
        if (!dev || !dev->GetMac()) {
            NS_LOG_ERROR("STA device " << i << " is invalid");
        }
    }

    // 移動模型
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(apNodes);

    mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                             "Bounds", RectangleValue(Rectangle(-50, 50, -50, 50)));
    mobility.Install(staNodes);

    // 網路協定堆疊
    InternetStackHelper stack;
    stack.Install(apNodes);
    stack.Install(staNodes);

    // IP位址分配
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterfaces = address.Assign(apDevices);
    Ipv4InterfaceContainer staInterfaces = address.Assign(staDevices);

    // 流量設定（Full-Queue Model）
    ApplicationContainer apps;
    for (uint32_t i = 0; i < staNodes.GetN(); ++i) {
        OnOffHelper onoff("ns3::UdpSocketFactory", 
                         InetSocketAddress(apInterfaces.GetAddress(i % nAp), 9));
        onoff.SetConstantRate(DataRate("1Mbps"), 1024);
        apps.Add(onoff.Install(staNodes.Get(i)));
    }
    apps.Start(Seconds(0.0));
    apps.Stop(Seconds(totalTime));

    // 流量監控
    FlowMonitorHelper flowMonitor;
    Ptr<FlowMonitor> monitor = flowMonitor.InstallAll();

    // 初始化數據文件
    std::ofstream outFile("throughput.csv");
    outFile << "Time,Total_Throughput_Mbps\n";
    
    std::ofstream userFile("user_distribution.csv");
    userFile << "Time,NodeA_Users,NodeB_Users\n";
    userFile << "0,16,16\n"; // 初始狀態

    // 移動事件設定
    UserMigrationHandler migrationHandler(apNodes, staNodes, apDevices, staDevices, nStaPerAp);
    migrationHandler.MigrateUsers(5, 0.25, 0.50);  // 5秒時，AP1->AP2 25%, AP2->AP1 50%
    migrationHandler.MigrateUsers(10, 0.50, 0.50); // 10秒時，AP1->AP2 50%, AP2->AP1 50%

    // 啟動吞吐量監控 - Add error handling
    Simulator::Schedule(Seconds(0.1), &ThroughputMonitor, monitor, std::ref(outFile));

    // 模擬執行
    NS_LOG_INFO("Starting simulation for " << totalTime << " seconds");
    Simulator::Stop(Seconds(totalTime));
    Simulator::Run();

    // 輸出結果
    NS_LOG_INFO("Simulation completed");
    monitor->CheckForLostPackets();
    Simulator::Destroy();
    return 0;
}
