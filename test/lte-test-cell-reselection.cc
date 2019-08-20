/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018 Fraunhofer ESK
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Vignesh Babu <ns3-dev@esk.fraunhofer.de>
 */

#include "lte-test-cell-reselection.h"

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/config-store-module.h"
#include "ns3/config-store.h"
#include <iostream>
#include <vector>
#include <stdio.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("LteCellReSelectionTest");

/*
 * Test Suite
 */
LteCellReselectionTestSuite::LteCellReselectionTestSuite ()
  : TestSuite ("lte-cell-reselection", SYSTEM)
{
  // REAL RRC PROTOCOL

  LteCellReselectionTestCase::ReselectionResults results;

  //normal mobility
  results.expectedQHyst = 4;
  results.expectedTimerDuration = Seconds (1);
  results.checkPoint = Seconds (11);
  results.cellIdBeforeLastReselection = 1;
  results.currentReselectedCellId = 2;
  results.expectedMobilitystate = "normal";

  AddTestCase (new LteCellReselectionTestCase ("real RRC, normal mobility", false, Vector(10, 0, 0), results), TestCase::QUICK);

  //medium mobility
  results.expectedQHyst = 2;
  results.expectedTimerDuration = Seconds (0.75);
  results.checkPoint = Seconds (41);
  results.cellIdBeforeLastReselection = 7;
  results.currentReselectedCellId = 8;
  results.expectedMobilitystate = "medium";

  //                                                                      isIdealRrc, velocity,     reselectionResults
  AddTestCase (new LteCellReselectionTestCase ("real RRC, medium mobility", false, Vector(25, 0, 0), results), TestCase::QUICK);

  //high mobility
  results.expectedQHyst = 0;
  results.expectedTimerDuration = Seconds (0.5);
  results.checkPoint = Seconds (36);
  results.cellIdBeforeLastReselection = 12;
  results.currentReselectedCellId = 13;
  results.expectedMobilitystate = "high";

  AddTestCase (new LteCellReselectionTestCase ("real RRC, high mobility", false, Vector(50, 0, 0), results), TestCase::QUICK);

} // end of LteCellReselectionTestSuite::LteCellReselectionTestSuite ()


static LteCellReselectionTestSuite g_lteCellReselectionTestSuite;



/*
 * Test Case
 */


LteCellReselectionTestCase::LteCellReselectionTestCase (
  std::string name, bool isIdealRrc,
  Vector velocity, ReselectionResults reselectionResults
  )
  : TestCase (name),
    m_isIdealRrc (isIdealRrc),
    m_velocity(velocity),
    m_reselectionResults (reselectionResults)
{
  NS_LOG_FUNCTION (this << GetName ());
  m_lastState=LteUeRrc::NUM_STATES;
  m_lastCampedOnCellId=0;
  m_qHyst=0;
  m_mobilityState="normal";
}


LteCellReselectionTestCase::~LteCellReselectionTestCase ()
{
  NS_LOG_FUNCTION (this << GetName ());
}


void
LteCellReselectionTestCase::DoRun ()
{
  NS_LOG_FUNCTION (this << GetName ());

  uint16_t numberOfUes = 1;
  uint16_t numberOfEnbs = 13;
  uint16_t numBearersPerUe = 1;
  double simTime = m_reselectionResults.checkPoint.GetSeconds()+3;
  double eNodeB_txPower = 43;
  double interSiteDistance = 150;

  Config::SetDefault ("ns3::LteHelper::UseIdealRrc", BooleanValue (m_isIdealRrc));
  if (!m_isIdealRrc)
    {
      Config::SetDefault ("ns3::LteHelper::UseIdealPrach", BooleanValue (false));
    }
  else
    {
      Config::SetDefault ("ns3::LteHelper::UseIdealPrach", BooleanValue (true));
    }

  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);

  lteHelper->SetPathlossModelType (TypeId::LookupByName ("ns3::LogDistancePropagationLossModel"));
  lteHelper->SetPathlossModelAttribute ("Exponent", DoubleValue (3.9));
  lteHelper->SetPathlossModelAttribute ("ReferenceLoss", DoubleValue (38.57)); //ref. loss in dB at 1m for 2.025GHz
  lteHelper->SetPathlossModelAttribute ("ReferenceDistance", DoubleValue (1));

  //----power related (equal for all base stations)----
  Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (eNodeB_txPower));
  Config::SetDefault ("ns3::LteUePhy::TxPower", DoubleValue (23));
  Config::SetDefault ("ns3::LteUePhy::NoiseFigure", DoubleValue (7));
  Config::SetDefault ("ns3::LteEnbPhy::NoiseFigure", DoubleValue (2));
  Config::SetDefault ("ns3::LteUePhy::EnableUplinkPowerControl", BooleanValue (true));
  Config::SetDefault ("ns3::LteUePowerControl::ClosedLoop", BooleanValue (true));
  Config::SetDefault ("ns3::LteUePowerControl::AccumulationEnabled", BooleanValue (true));

  lteHelper->SetHandoverAlgorithmType ("ns3::A3RsrpHandoverAlgorithm");
  lteHelper->SetHandoverAlgorithmAttribute ("Hysteresis", DoubleValue (3.0)); //Handover margin (0-15 dB range).MAX=15 dB
  lteHelper->SetHandoverAlgorithmAttribute ("TimeToTrigger", TimeValue (MilliSeconds (256)));

  //----frequency related----
  lteHelper->SetEnbDeviceAttribute ("DlEarfcn", UintegerValue (100)); //2120MHz
  lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (18100)); //1930MHz
  lteHelper->SetEnbDeviceAttribute ("DlBandwidth", UintegerValue (25)); //5MHz
  lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (25)); //5MHz

  //----others----
  lteHelper->SetSchedulerType ("ns3::PfFfMacScheduler");
  Config::SetDefault ("ns3::LteAmc::AmcModel", EnumValue (LteAmc::PiroEW2010));
  Config::SetDefault ("ns3::LteAmc::Ber", DoubleValue (0.01));
  Config::SetDefault ("ns3::PfFfMacScheduler::HarqEnabled", BooleanValue (true));

  Config::SetDefault ("ns3::LteEnbMac::DynamicTimerConfiguration", BooleanValue (false));

  //Cell reselection parameters
  Config::SetDefault ("ns3::LteUeRrc::CellReselectionTimer", TimeValue (Seconds (1)));//1s, TreselectionEUTRA
  Config::SetDefault ("ns3::LteUeRrc::SIntraSearch", UintegerValue (29)); //(29*2)dB, s-IntraSearch
  Config::SetDefault ("ns3::LteUeRrc::QHyst", DoubleValue (4));//4 dB, Qhyst
  Config::SetDefault ("ns3::LteUeRrc::QOffsetCell", DoubleValue (0));//0 dB, Qoffset(s,n)
  Config::SetDefault ("ns3::LteUeRrc::TCrMax", TimeValue (Seconds (30)));//30s, TCRmax

  // Create the internet
  Ptr<Node> pgw = epcHelper->GetPgwNode ();
  // Create a single RemoteHost0x18ab460
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<
      Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  // Create Nodes: eNodeB and UE
  NodeContainer enbNodes;
  NodeContainer ueNodes;
  enbNodes.Create (numberOfEnbs);
  ueNodes.Create (numberOfUes);

  //Mobility
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

  for (uint16_t i = 0; i < numberOfEnbs; i++)
    {
      positionAlloc->Add (Vector (interSiteDistance * i, 0, 0));
    }
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator (positionAlloc);
  mobility.Install (enbNodes);

  Ptr<ListPositionAllocator> positionAlloc1 = CreateObject<ListPositionAllocator> ();

  for (int i = 0; i < numberOfUes; i++)
    {
      positionAlloc1->Add (Vector(0,0,0));
    }

  mobility.SetPositionAllocator (positionAlloc1);
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (ueNodes);

  for (int i = 0; i < numberOfUes; i++)
    {
      ueNodes.Get (i)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (m_velocity);
    }

  // Install LTE Devices in eNB and UEs
  NetDeviceContainer enbDevs;
  NetDeviceContainer ueDevs;

  enbDevs = lteHelper->InstallEnbDevice (enbNodes);
  ueDevs = lteHelper->InstallUeDevice (ueNodes);

  // Install the IP stack on the UEs
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIfaces;
  ueIpIfaces = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueDevs));

  // Attach a UE to a eNB
  lteHelper->Attach (ueDevs);

  // Install and start applications on UEs and remote host
  uint16_t dlPort = 10000;
  uint16_t ulPort = 20000;
  Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<
      UniformRandomVariable> ();
  startTimeSeconds->SetAttribute ("Min", DoubleValue (0));
  startTimeSeconds->SetAttribute ("Max", DoubleValue (0.010));
  double interPacketInterval = 1000;

  for (uint32_t u = 0; u < numberOfUes; ++u)
    {
      Ptr<Node> ue = ueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ue->GetObject<
          Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

      for (uint32_t b = 0; b < numBearersPerUe; ++b)
        {

          ApplicationContainer ulClientApps;
          ApplicationContainer ulServerApps;
          ApplicationContainer dlClientApps;
          ApplicationContainer dlServerApps;

          ++dlPort;
          ++ulPort;

          NS_LOG_LOGIC("installing UDP DL app for UE " << u+1);
          UdpClientHelper dlClientHelper (ueIpIfaces.GetAddress (u), dlPort);
          dlClientHelper.SetAttribute ("Interval", TimeValue (MilliSeconds (interPacketInterval)));
          dlClientHelper.SetAttribute ("MaxPackets", UintegerValue (1000000));
          dlClientApps.Add (dlClientHelper.Install (remoteHost));

          PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dlPort));
          dlServerApps.Add (dlPacketSinkHelper.Install (ue));

          NS_LOG_LOGIC("installing UDP UL app for UE " << u+1);
          UdpClientHelper ulClientHelper (remoteHostAddr, ulPort);
          ulClientHelper.SetAttribute ("Interval", TimeValue (MilliSeconds (interPacketInterval)));
          ulClientHelper.SetAttribute ("MaxPackets", UintegerValue (1000000));
          ulClientApps.Add (ulClientHelper.Install (ue));

          PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
          ulServerApps.Add (ulPacketSinkHelper.Install (remoteHost));

          Ptr<EpcTft> tft = Create<EpcTft> ();
          EpcTft::PacketFilter dlpf;
          dlpf.localPortStart = dlPort;
          dlpf.localPortEnd = dlPort;
          tft->Add (dlpf);
          EpcTft::PacketFilter ulpf;
          ulpf.remotePortStart = ulPort;
          ulpf.remotePortEnd = ulPort;
          tft->Add (ulpf);
          EpsBearer bearer (EpsBearer::NGBR_IMS);
          lteHelper->ActivateDedicatedEpsBearer (ueDevs.Get (u), bearer, tft);

          ulServerApps.Start (Seconds (m_reselectionResults.checkPoint.GetSeconds()+1));
          ulClientApps.Start (Seconds (m_reselectionResults.checkPoint.GetSeconds()+1));
          dlServerApps.Start (Seconds (m_reselectionResults.checkPoint.GetSeconds()+1));
          dlClientApps.Start (Seconds (m_reselectionResults.checkPoint.GetSeconds()+1));
        } // end for b
    }

  // Add X2 inteface
  if (numberOfEnbs > 1)
    lteHelper->AddX2Interface (enbNodes);

  Ptr<LteUeNetDevice> ueLteDevice = ueDevs.Get(0)->GetObject<LteUeNetDevice>();
  Simulator::Schedule (m_reselectionResults.checkPoint,
                             &LteCellReselectionTestCase::CheckPoint,
                             this, ueLteDevice,
                             m_reselectionResults);

  // connect custom trace sinks for RRC connection establishment and handover notification
   Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
                    MakeCallback (&LteCellReselectionTestCase::ConnectionEstablishedCallback, this));
   Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/StateTransition",
                                     MakeCallback (&LteCellReselectionTestCase::StateTransitionCallback, this));
   Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/CellReselection",
                      MakeCallback (&LteCellReselectionTestCase::CellReselectionCallback, this));
   Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/UeMobilityStateChanged",
                      MakeCallback (&LteCellReselectionTestCase::UeMobilityStateChangedCallback, this));

   Simulator::Stop (Seconds (simTime));

   Simulator::Run ();

   NS_TEST_ASSERT_MSG_EQ(m_lastState, LteUeRrc::CONNECTED_NORMALLY,
                         "UE " << ueLteDevice->GetImsi ()
                                 << " is not at CONNECTED_NORMALLY state");
  NS_LOG_INFO ("Simulation ends");
  Simulator::Destroy ();

} // end of void LteCellReselectionTestCase::DoRun ()


void
LteCellReselectionTestCase::CheckPoint (Ptr<LteUeNetDevice> ueDev, ReselectionResults reselectionResults)
{
  uint16_t actualCellId = ueDev->GetRrc ()->GetCellId ();

  NS_TEST_ASSERT_MSG_EQ(reselectionResults.cellIdBeforeLastReselection, m_lastCampedOnCellId,
                        "UE with IMSI " << ueDev->GetImsi () << " was camped on an unexpected cell");
  NS_TEST_ASSERT_MSG_EQ(reselectionResults.currentReselectedCellId, actualCellId,
                        "UE with IMSI " << ueDev->GetImsi () << " is camped on an unexpected cell");
  NS_TEST_ASSERT_MSG_EQ(reselectionResults.expectedQHyst, m_qHyst, "Unexpected hysteresis value");
  NS_TEST_ASSERT_MSG_EQ(reselectionResults.expectedTimerDuration, m_reselectionTimerDuration,
                        "Unexpected reselection timer value");
  NS_TEST_ASSERT_MSG_EQ(reselectionResults.expectedMobilitystate, m_mobilityState,
                         "Unexpected mobility state");
}


void
LteCellReselectionTestCase::StateTransitionCallback (
  std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti,
  LteUeRrc::State oldState, LteUeRrc::State newState)
{
  NS_LOG_FUNCTION (this << imsi << cellId << rnti << oldState << newState);
  m_lastState = newState;
}


void
LteCellReselectionTestCase::ConnectionEstablishedCallback (
  std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << imsi << cellId << rnti);
}

void
LteCellReselectionTestCase::CellReselectionCallback (std::string context, uint64_t imsi, uint16_t rnti,
                                                      LteUeRrc::ReselectionInfo reselectionInfo)
{
  NS_LOG_FUNCTION (this << imsi << rnti);
  m_lastCampedOnCellId= reselectionInfo.currentCellId;
  m_qHyst=reselectionInfo.hyst;
  m_reselectionTimerDuration=reselectionInfo.reselectionTimerDuration;
}

void
LteCellReselectionTestCase::UeMobilityStateChangedCallback (std::string context, uint64_t imsi, Time reselectionTimerDuration,
                                                            double qHyst, std::string type)
{
  NS_LOG_FUNCTION (this << imsi);
  m_mobilityState = type;
}



