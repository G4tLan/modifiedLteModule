/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 * Copyright (c) 2015, University of Padova, Dep. of Information Engineering, SIGNET lab.
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
 * Author: Jaume Nin <jnin@cttc.cat>
 *         Nicola Baldo <nbaldo@cttc.cat>
 *
 * Modified by Michele Polese <michele.polese@gmail.com>
 *     (support for RRC_CONNECTED->RRC_IDLE state transition)
 *
 * Modified by Vignesh Babu <ns3-dev@esk.fraunhofer.de> (support for Paging)
 */


#include "epc-sgw-pgw-application.h"
#include "ns3/log.h"
#include "ns3/mac48-address.h"
#include "ns3/ipv4.h"
#include "ns3/ipv6.h"
#include "ns3/ipv6-header.h"
#include "ns3/inet-socket-address.h"
#include "ns3/epc-gtpu-header.h"
#include "ns3/abort.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("EpcSgwPgwApplication");

NS_OBJECT_ENSURE_REGISTERED (EpcSgwPgwApplication);

/////////////////////////
// UeInfo
/////////////////////////


EpcSgwPgwApplication::UeInfo::UeInfo ()
{
  NS_LOG_FUNCTION (this);
}

bool
EpcSgwPgwApplication::UeInfo::AddBearer (Ptr<EpcTft> tft, uint8_t bearerId, uint32_t teid)
{
  NS_LOG_FUNCTION (this << tft << teid);
//  m_teidByBearerIdMap[bearerId] = teid;
  /**
   * Bug:
   * Bearers are added when eNodeB sends InitialUeMessage to EPC (upon receiving
   * RrcConnectionRequest from the UE). Due to random access being attempted multiple
   * times (due to expiration of timers or loss of messages, before a successful connection is
   * established), eNodeB sends multiple (InitialUeMessage)s to EPC for same UE
   * (due to receiving multiple (RrcConnectionRequest)s from the UE). Thus it causes many
   * TFTs to be added to the classifier for the same bearer instead of having
   * one TFT per bearer.
   *
   * Bug Fix:
   * Hence, add the bearer to m_teidByBearerIdMap and m_tftClassifier
   * only if it is not present.
   * 
   */
  std::pair<std::map<uint8_t, uint32_t >::iterator,bool> ret;
  ret=m_teidByBearerIdMap.insert ( std::pair<uint8_t, uint32_t>(bearerId, teid) );
  if(ret.second==false)
    {
     return false; //If corresponding bearer already exists in the map, exit from method with false value
    }
  m_tftClassifier.Add (tft, teid);
  return true;
}

void
EpcSgwPgwApplication::UeInfo::RemoveBearer (uint8_t bearerId)
{
  NS_LOG_FUNCTION (this << bearerId);
  std::map<uint8_t, uint32_t >::iterator it = m_teidByBearerIdMap.find (bearerId);
  if(it != m_teidByBearerIdMap.end ())
  m_tftClassifier.Delete(it->second);//delete tft if not yet deleted
  m_teidByBearerIdMap.erase (bearerId);
}

uint32_t
EpcSgwPgwApplication::UeInfo::Classify (Ptr<Packet> p)
{
  NS_LOG_FUNCTION (this << p);
  // we hardcode DOWNLINK direction since the PGW is espected to
  // classify only downlink packets (uplink packets will go to the
  // internet without any classification). 
  return m_tftClassifier.Classify (p, EpcTft::DOWNLINK);
}

Ipv4Address 
EpcSgwPgwApplication::UeInfo::GetEnbAddr ()
{
  return m_enbAddr;
}

void
EpcSgwPgwApplication::UeInfo::SetEnbAddr (Ipv4Address enbAddr)
{
  m_enbAddr = enbAddr;
}

Ipv4Address 
EpcSgwPgwApplication::UeInfo::GetUeAddr ()
{
  return m_ueAddr;
}

void
EpcSgwPgwApplication::UeInfo::SetUeAddr (Ipv4Address ueAddr)
{
  m_ueAddr = ueAddr;
}

Ipv6Address 
EpcSgwPgwApplication::UeInfo::GetUeAddr6 ()
{
  return m_ueAddr6;
}

void
EpcSgwPgwApplication::UeInfo::SetUeAddr6 (Ipv6Address ueAddr)
{
  m_ueAddr6 = ueAddr;
}

/////////////////////////
// EpcSgwPgwApplication
/////////////////////////


TypeId
EpcSgwPgwApplication::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::EpcSgwPgwApplication")
    .SetParent<Object> ()
    .SetGroupName("Lte")
	.AddAttribute ("MaxDlBufferSize",
	               "Maximum Size of the Downlink Packet Buffer (in Bytes)",
	               UintegerValue (10 * 1024),
	               MakeUintegerAccessor (&EpcSgwPgwApplication::m_maxDlBufferSize),
	               MakeUintegerChecker<uint32_t> ())
    .AddTraceSource ("RxFromTun",
                     "Receive data packets from internet in Tunnel net device",
                     MakeTraceSourceAccessor (&EpcSgwPgwApplication::m_rxTunPktTrace),
                     "ns3::EpcSgwPgwApplication::RxTracedCallback")
    .AddTraceSource ("RxFromS1u",
                     "Receive data packets from S1 U Socket",
                     MakeTraceSourceAccessor (&EpcSgwPgwApplication::m_rxS1uPktTrace),
                     "ns3::EpcSgwPgwApplication::RxTracedCallback")
    ;
  return tid;
}

void
EpcSgwPgwApplication::DoDispose ()
{
  NS_LOG_FUNCTION (this);
  m_s1uSocket->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
  m_s1uSocket = 0;
  delete (m_s11SapSgw);
}

  
EpcSgwPgwApplication::EpcSgwPgwApplication (const Ptr<VirtualNetDevice> tunDevice, const Ptr<Socket> s1uSocket)
  : m_s1uSocket (s1uSocket),
    m_tunDevice (tunDevice),
    m_gtpuUdpPort (2152), // fixed by the standard
    m_teidCount (0),
    m_s11SapMme (0)
{
  NS_LOG_FUNCTION (this << tunDevice << s1uSocket);
  m_s1uSocket->SetRecvCallback (MakeCallback (&EpcSgwPgwApplication::RecvFromS1uSocket, this));
  m_s11SapSgw = new MemberEpcS11SapSgw<EpcSgwPgwApplication> (this);
}

  
EpcSgwPgwApplication::~EpcSgwPgwApplication ()
{
  NS_LOG_FUNCTION (this);
}


bool
EpcSgwPgwApplication::RecvFromTunDevice (Ptr<Packet> packet, const Address& source, const Address& dest, uint16_t protocolNumber)
{
  NS_LOG_FUNCTION (this << source << dest << packet << packet->GetSize ());
  m_rxTunPktTrace (packet->Copy ());
  Ptr<Packet> pCopy = packet->Copy ();

  uint8_t ipType;
  pCopy->CopyData (&ipType, 1);
  ipType = (ipType>>4) & 0x0f;

  // get IP address of UE
  if (ipType == 0x04)
    {
      Ipv4Header ipv4Header;
      pCopy->RemoveHeader (ipv4Header);
      Ipv4Address ueAddr =  ipv4Header.GetDestination ();
      NS_LOG_LOGIC ("packet addressed to UE " << ueAddr);
      // find corresponding UeInfo address
      std::map<Ipv4Address, Ptr<UeInfo> >::iterator it = m_ueInfoByAddrMap.find (ueAddr);
      if (it == m_ueInfoByAddrMap.end ())
        {        
          NS_LOG_WARN ("unknown UE address " << ueAddr);
        }
      else
        {
          Ipv4Address enbAddr = it->second->GetEnbAddr ();      
          uint32_t teid = it->second->Classify (packet);   
          if (teid == 0)
            {
              NS_LOG_WARN ("no matching bearer for this packet");
              NS_LOG_INFO("Buffer the packet directed to UE: " << it->second->GetImsi());
              it->second->AddPacketToBuffer (packet);//Buffer the packets and start paging procedure
            }
          else
            {
              SendToS1uSocket (packet, enbAddr, teid);
            }
        }
    }
  else if (ipType == 0x06)
    {
      Ipv6Header ipv6Header;
      pCopy->RemoveHeader (ipv6Header);
      Ipv6Address ueAddr =  ipv6Header.GetDestinationAddress ();
      NS_LOG_LOGIC ("packet addressed to UE " << ueAddr);
      // find corresponding UeInfo address
      std::map<Ipv6Address, Ptr<UeInfo> >::iterator it = m_ueInfoByAddrMap6.find (ueAddr);
      if (it == m_ueInfoByAddrMap6.end ())
        {        
          NS_LOG_WARN ("unknown UE address " << ueAddr);
        }
      else
        {
          Ipv4Address enbAddr = it->second->GetEnbAddr ();      
          uint32_t teid = it->second->Classify (packet);   
          if (teid == 0)
            {
              NS_LOG_WARN ("no matching bearer for this packet");  
              NS_LOG_INFO("Buffer the packet directed to UE: " << it->second->GetImsi());
              it->second->AddPacketToBuffer (packet);//Buffer the packets and start paging procedure                 
            }
          else
            {
              SendToS1uSocket (packet, enbAddr, teid);
            }
        }
    }
  else
    {
      NS_ABORT_MSG ("EpcSgwPgwApplication::RecvFromTunDevice - Unknown IP type...");
    }

  // there is no reason why we should notify the TUN
  // VirtualNetDevice that he failed to send the packet: if we receive
  // any bogus packet, it will just be silently discarded.
  const bool succeeded = true;
  return succeeded;
}

void 
EpcSgwPgwApplication::RecvFromS1uSocket (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);  
  NS_ASSERT (socket == m_s1uSocket);
  Ptr<Packet> packet = socket->Recv ();
  GtpuHeader gtpu;
  packet->RemoveHeader (gtpu);
  uint32_t teid = gtpu.GetTeid ();

  SendToTunDevice (packet, teid);

  m_rxS1uPktTrace (packet->Copy ());
}

void 
EpcSgwPgwApplication::SendToTunDevice (Ptr<Packet> packet, uint32_t teid)
{
  NS_LOG_FUNCTION (this << packet << teid);
  NS_LOG_LOGIC (" packet size: " << packet->GetSize () << " bytes");

  uint8_t ipType;
  packet->CopyData (&ipType, 1);
  ipType = (ipType>>4) & 0x0f;

  if (ipType == 0x04)
    {
      m_tunDevice->Receive (packet, 0x0800, m_tunDevice->GetAddress (), m_tunDevice->GetAddress (), NetDevice::PACKET_HOST);
    }
  else if (ipType == 0x06)
    {
      m_tunDevice->Receive (packet, 0x86DD, m_tunDevice->GetAddress (), m_tunDevice->GetAddress (), NetDevice::PACKET_HOST);
    }
  else
    {
      NS_ABORT_MSG ("EpcSgwPgwApplication::SendToTunDevice - Unknown IP type...");
    }
}

void 
EpcSgwPgwApplication::SendToS1uSocket (Ptr<Packet> packet, Ipv4Address enbAddr, uint32_t teid)
{
  NS_LOG_FUNCTION (this << packet << enbAddr << teid);

  GtpuHeader gtpu;
  gtpu.SetTeid (teid);
  // From 3GPP TS 29.281 v10.0.0 Section 5.1
  // Length of the payload + the non obligatory GTP-U header
  gtpu.SetLength (packet->GetSize () + gtpu.GetSerializedSize () - 8);  
  packet->AddHeader (gtpu);
  uint32_t flags = 0;
  m_s1uSocket->SendTo (packet, flags, InetSocketAddress (enbAddr, m_gtpuUdpPort));
}


void 
EpcSgwPgwApplication::SetS11SapMme (EpcS11SapMme * s)
{
  m_s11SapMme = s;
}

EpcS11SapSgw* 
EpcSgwPgwApplication::GetS11SapSgw ()
{
  return m_s11SapSgw;
}

void 
EpcSgwPgwApplication::AddEnb (uint16_t cellId, Ipv4Address enbAddr, Ipv4Address sgwAddr)
{
  NS_LOG_FUNCTION (this << cellId << enbAddr << sgwAddr);
  EnbInfo enbInfo;
  enbInfo.enbAddr = enbAddr;
  enbInfo.sgwAddr = sgwAddr;
  m_enbInfoByCellId[cellId] = enbInfo;
}

void 
EpcSgwPgwApplication::AddUe (uint64_t imsi)
{
  NS_LOG_FUNCTION (this << imsi);
  Ptr<UeInfo> ueInfo = Create<UeInfo> ();
  m_ueInfoByImsiMap[imsi] = ueInfo;
//Set the initial values for the paging parameters
  ueInfo->m_maxDlBufferSize =m_maxDlBufferSize;
  ueInfo->m_dlBufferSize=0;
  ueInfo->m_dlDataNotificationFlag=false;
  ueInfo->m_imsi=imsi;
  ueInfo->m_epcSgwPgwApplication=this;
}

void 
EpcSgwPgwApplication::SetUeAddress (uint64_t imsi, Ipv4Address ueAddr)
{
  NS_LOG_FUNCTION (this << imsi << ueAddr);
  std::map<uint64_t, Ptr<UeInfo> >::iterator ueit = m_ueInfoByImsiMap.find (imsi);
  NS_ASSERT_MSG (ueit != m_ueInfoByImsiMap.end (), "unknown IMSI " << imsi); 
  m_ueInfoByAddrMap[ueAddr] = ueit->second;
  ueit->second->SetUeAddr (ueAddr);
}

void 
EpcSgwPgwApplication::SetUeAddress6 (uint64_t imsi, Ipv6Address ueAddr)
{
  NS_LOG_FUNCTION (this << imsi << ueAddr);
  std::map<uint64_t, Ptr<UeInfo> >::iterator ueit = m_ueInfoByImsiMap.find (imsi);
  NS_ASSERT_MSG (ueit != m_ueInfoByImsiMap.end (), "unknown IMSI " << imsi); 
  m_ueInfoByAddrMap6[ueAddr] = ueit->second;
  ueit->second->SetUeAddr6 (ueAddr);
}

void 
EpcSgwPgwApplication::DoCreateSessionRequest (EpcS11SapSgw::CreateSessionRequestMessage req)
{
  NS_LOG_FUNCTION (this << req.imsi);
  std::map<uint64_t, Ptr<UeInfo> >::iterator ueit = m_ueInfoByImsiMap.find (req.imsi);
  NS_ASSERT_MSG (ueit != m_ueInfoByImsiMap.end (), "unknown IMSI " << req.imsi); 
  uint16_t cellId = req.uli.gci;
  std::map<uint16_t, EnbInfo>::iterator enbit = m_enbInfoByCellId.find (cellId);
  NS_ASSERT_MSG (enbit != m_enbInfoByCellId.end (), "unknown CellId " << cellId); 
  Ipv4Address enbAddr = enbit->second.enbAddr;
  ueit->second->SetEnbAddr (enbAddr);

  EpcS11SapMme::CreateSessionResponseMessage res;
  res.teid = req.imsi; // trick to avoid the need for allocating TEIDs on the S11 interface

  for (std::list<EpcS11SapSgw::BearerContextToBeCreated>::iterator bit = req.bearerContextsToBeCreated.begin ();
       bit != req.bearerContextsToBeCreated.end ();
       ++bit)
    {
      // simple sanity check. If you ever need more than 4M teids
      // throughout your simulation, you'll need to implement a smarter teid
      // management algorithm. 
      NS_ABORT_IF (m_teidCount == 0xFFFFFFFF);
      uint32_t teid = ++m_teidCount;  
      if(!ueit->second->AddBearer (bit->tft, bit->epsBearerId, teid))
        --m_teidCount;//if bearer is already added before, don't increment TEID count

      EpcS11SapMme::BearerContextCreated bearerContext;
      bearerContext.sgwFteid.teid =  ueit->second->GetTeid(bit->epsBearerId);//get TEID for a bearer from epsBearerId
      bearerContext.sgwFteid.address = enbit->second.sgwAddr;
      bearerContext.epsBearerId =  bit->epsBearerId; 
      bearerContext.bearerLevelQos = bit->bearerLevelQos; 
      bearerContext.tft = bit->tft;
      res.bearerContextsCreated.push_back (bearerContext);
    }
  m_s11SapMme->CreateSessionResponse (res);
  
}

void 
EpcSgwPgwApplication::DoModifyBearerRequest (EpcS11SapSgw::ModifyBearerRequestMessage req)
{
  NS_LOG_FUNCTION (this << req.teid);
  uint64_t imsi = req.teid; // trick to avoid the need for allocating TEIDs on the S11 interface
  std::map<uint64_t, Ptr<UeInfo> >::iterator ueit = m_ueInfoByImsiMap.find (imsi);
  NS_ASSERT_MSG (ueit != m_ueInfoByImsiMap.end (), "unknown IMSI " << imsi); 
  uint16_t cellId = req.uli.gci;
  std::map<uint16_t, EnbInfo>::iterator enbit = m_enbInfoByCellId.find (cellId);
  NS_ASSERT_MSG (enbit != m_enbInfoByCellId.end (), "unknown CellId " << cellId); 
  Ipv4Address enbAddr = enbit->second.enbAddr;
  ueit->second->SetEnbAddr (enbAddr);
  // no actual bearer modification: for now we just support the minimum needed for path switch request (handover)
  EpcS11SapMme::ModifyBearerResponseMessage res;
  res.teid = imsi; // trick to avoid the need for allocating TEIDs on the S11 interface
  res.cause = EpcS11SapMme::ModifyBearerResponseMessage::REQUEST_ACCEPTED;
  m_s11SapMme->ModifyBearerResponse (res);
}
 
void
EpcSgwPgwApplication::DoDeleteBearerCommand (EpcS11SapSgw::DeleteBearerCommandMessage req)
{
  NS_LOG_FUNCTION (this << req.teid);
  uint64_t imsi = req.teid; // trick to avoid the need for allocating TEIDs on the S11 interface
  std::map<uint64_t, Ptr<UeInfo> >::iterator ueit = m_ueInfoByImsiMap.find (imsi);
  NS_ASSERT_MSG (ueit != m_ueInfoByImsiMap.end (), "unknown IMSI " << imsi);

  EpcS11SapMme::DeleteBearerRequestMessage res;
  res.teid = imsi;

  for (std::list<EpcS11SapSgw::BearerContextToBeRemoved>::iterator bit = req.bearerContextsToBeRemoved.begin ();
       bit != req.bearerContextsToBeRemoved.end ();
       ++bit)
    {
      EpcS11SapMme::BearerContextRemoved bearerContext;
      bearerContext.epsBearerId =  bit->epsBearerId;
      res.bearerContextsRemoved.push_back (bearerContext);
    }
  //schedules Delete Bearer Request towards MME
  m_s11SapMme->DeleteBearerRequest (res);
}

void
EpcSgwPgwApplication::DoDeleteBearerResponse (EpcS11SapSgw::DeleteBearerResponseMessage req)
{
  NS_LOG_FUNCTION (this << req.teid);
  uint64_t imsi = req.teid; // trick to avoid the need for allocating TEIDs on the S11 interface
  std::map<uint64_t, Ptr<UeInfo> >::iterator ueit = m_ueInfoByImsiMap.find (imsi);
  NS_ASSERT_MSG (ueit != m_ueInfoByImsiMap.end (), "unknown IMSI " << imsi);

  for (std::list<EpcS11SapSgw::BearerContextRemovedSgwPgw>::iterator bit = req.bearerContextsRemoved.begin ();
       bit != req.bearerContextsRemoved.end ();
       ++bit)
    {
      //Function to remove de-activated bearer contexts from S-Gw and P-Gw side
      ueit->second->RemoveBearer (bit->epsBearerId);
    }
  //clear buffer when removing a bearer of an UE
  ueit->second->m_dlDataNotificationFlag=false;
  ueit->second->m_dlBufferSize=0;
  ueit->second->m_dlBuffer.clear();
}

void
EpcSgwPgwApplication::UeInfo::AddPacketToBuffer (Ptr<Packet> packet)
{
  NS_LOG_FUNCTION(this << packet);

  if (m_dlBufferSize + packet->GetSize () <= m_maxDlBufferSize)
    {
      NS_LOG_LOGIC("Dl Buffer: New packet added");
      m_dlBuffer.push_back (packet);
      m_dlBufferSize += packet->GetSize ();
      NS_LOG_LOGIC("NumOfPackets = " << m_dlBuffer.size());
      NS_LOG_LOGIC("DlBufferSize = " << m_dlBufferSize);
      NS_LOG_LOGIC("MaxDlBufferSize = " << m_maxDlBufferSize);
      if (m_dlDataNotificationFlag == false)
        {
          //Send the downlink data notification message to the MME, see 3GPP TS 24.301 5.6.2
          EpcS11SapMme::DownlinkDataNotificationMessage msg;
          msg.imsi = GetImsi ();
          m_epcSgwPgwApplication->m_s11SapMme->DownlinkDataNotification (msg);
          m_dlDataNotificationFlag = true;
        }
    }
  else
    {
      // Discard Application packet
      NS_LOG_LOGIC("DlBuffer is full. Application packet discarded");
      NS_LOG_LOGIC("MaxDlBufferSize = " << m_maxDlBufferSize);
      NS_LOG_LOGIC("DlBufferSize    = " << m_dlBufferSize);
      NS_LOG_LOGIC("packet size     = " << packet->GetSize ());
    }
}

uint64_t
EpcSgwPgwApplication::UeInfo::GetImsi ()
{
  return m_imsi;
}

uint32_t EpcSgwPgwApplication::UeInfo::GetTeid(uint8_t epsBearerId)
{
  std::map<uint8_t, uint32_t >::iterator it = m_teidByBearerIdMap.find (epsBearerId);
  NS_ASSERT_MSG (it != m_teidByBearerIdMap.end (), "unknown epsBearerId " << epsBearerId);
  return it->second;
}

void
EpcSgwPgwApplication::DoSendBufferedDlPackets (uint64_t imsi)
{
  NS_LOG_FUNCTION(this << imsi);
  std::map<uint64_t, Ptr<UeInfo> >::iterator it = m_ueInfoByImsiMap.find (imsi);
  if (it == m_ueInfoByImsiMap.end ())
    {
      NS_LOG_WARN("unknown UE " << imsi);
    }
  else
    {
      Ipv4Address enbAddr = it->second->GetEnbAddr ();
      while (!it->second->m_dlBuffer.empty ())
        {
          uint32_t teid = it->second->Classify (it->second->m_dlBuffer.front ());
          if (teid == 0)
            {
              NS_LOG_WARN("no matching bearer for this packet");
            }
          else
            {
              SendToS1uSocket (it->second->m_dlBuffer.front (), enbAddr, teid);

            }
          it->second->m_dlBufferSize -= it->second->m_dlBuffer.front ()->GetSize ();
          it->second->m_dlBuffer.pop_front ();
        }
      it->second->m_dlDataNotificationFlag = false;
    }
}

void
EpcSgwPgwApplication::DoDiscardBufferedDlPackets (uint64_t imsi)
{
  NS_LOG_FUNCTION(this << imsi);
  std::map<uint64_t, Ptr<UeInfo> >::iterator it = m_ueInfoByImsiMap.find (imsi);
  if (it == m_ueInfoByImsiMap.end ())
    {
      NS_LOG_WARN("unknown UE " << imsi);
    }
  else
    {
      it->second->m_dlBuffer.clear ();
      it->second->m_dlBufferSize = 0;
      it->second->m_dlDataNotificationFlag = false;
    }
}

}  // namespace ns3
