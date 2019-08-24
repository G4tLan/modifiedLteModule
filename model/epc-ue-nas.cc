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
 * Author: Nicola Baldo <nbaldo@cttc.es>
 *
 * Modified by Michele Polese <michele.polese@gmail.com>
 *     (support for RRC_CONNECTED->RRC_IDLE state transition)
 *
 * Modified by Vignesh Babu <ns3-dev@esk.fraunhofer.de>
 *        (support for Paging;
 *    integrated the RRC_CONNECTED->RRC_IDLE
 *    state transition (taken from Lena-plus(work of Michele Polese)) and also enhanced the module)
 */

#include <ns3/fatal-error.h>
#include <ns3/log.h>

#include <ns3/epc-helper.h>

#include "lte-enb-net-device.h"
#include "epc-ue-nas.h"
#include "lte-as-sap.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("EpcUeNas");



/// Map each of UE NAS states to its string representation.
static const std::string g_ueNasStateName[EpcUeNas::NUM_STATES] =
{
  "OFF",
  "ATTACHING",
  "IDLE_REGISTERED",
  "CONNECTING_TO_EPC",
  "ACTIVE"
};

/**
 * \param s The UE NAS state.
 * \return The string representation of the given state.
 */
static inline const std::string & ToString (EpcUeNas::State s)
{
  return g_ueNasStateName[s];
}




NS_OBJECT_ENSURE_REGISTERED (EpcUeNas);

EpcUeNas::EpcUeNas ()
  : m_state (OFF),
    m_csgId (0),
    m_asSapProvider (0),
    m_bidCounter (0)
{
  NS_LOG_FUNCTION (this);
  m_asSapUser = new MemberLteAsSapUser<EpcUeNas> (this);
  //paging parameters
  m_UlBufferSize = 0;
}


EpcUeNas::~EpcUeNas ()
{
  NS_LOG_FUNCTION (this);
}

void
EpcUeNas::DoDispose ()
{
  NS_LOG_FUNCTION (this);
  delete m_asSapUser;
}

TypeId
EpcUeNas::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::EpcUeNas")
    .SetParent<Object> ()
    .SetGroupName ("Lte")
    .AddConstructor<EpcUeNas> ()
    .AddAttribute ("MaxUlBufferSize",
                       "Maximum Size of the Uplink Packet Buffer (in Bytes)",
                       UintegerValue (10 * 1024),
                       MakeUintegerAccessor (&EpcUeNas::m_maxUlBufferSize),
                       MakeUintegerChecker<uint32_t> ())
    .AddTraceSource ("StateTransition",
                     "fired upon every UE NAS state transition",
                     MakeTraceSourceAccessor (&EpcUeNas::m_stateTransitionCallback),
                     "ns3::EpcUeNas::StateTracedCallback")
  ;
  return tid;
}

void
EpcUeNas::SetDevice (Ptr<NetDevice> dev)
{
  NS_LOG_FUNCTION (this << dev);
  m_device = dev;
}

void
EpcUeNas::SetImsi (uint64_t imsi)
{
  NS_LOG_FUNCTION (this << imsi);
  m_imsi = imsi;
}

void
EpcUeNas::SetCsgId (uint32_t csgId)
{
  NS_LOG_FUNCTION (this << csgId);
  m_csgId = csgId;
  m_asSapProvider->SetCsgWhiteList (csgId);
}

uint32_t
EpcUeNas::GetCsgId () const
{
  NS_LOG_FUNCTION (this);
  return m_csgId;
}

void
EpcUeNas::SetAsSapProvider (LteAsSapProvider* s)
{
  NS_LOG_FUNCTION (this << s);
  m_asSapProvider = s;
}

LteAsSapUser*
EpcUeNas::GetAsSapUser ()
{
  NS_LOG_FUNCTION (this);
  return m_asSapUser;
}

void
EpcUeNas::SetForwardUpCallback (Callback <void, Ptr<Packet> > cb)
{
  NS_LOG_FUNCTION (this);
  m_forwardUpCallback = cb;
}

void
EpcUeNas::StartCellSelection (uint32_t dlEarfcn)
{
  NS_LOG_FUNCTION (this << dlEarfcn);
  m_asSapProvider->StartCellSelection (dlEarfcn);
}

void
EpcUeNas::Connect ()
{
  NS_LOG_FUNCTION (this);

  // tell RRC to go into connected mode
  m_asSapProvider->Connect ();
}

void
EpcUeNas::Connect (uint16_t cellId, uint32_t dlEarfcn)
{
  NS_LOG_FUNCTION (this << cellId << dlEarfcn);

  // force the UE RRC to be camped on a specific eNB
  m_asSapProvider->ForceCampedOnEnb (cellId, dlEarfcn);

  // tell RRC to go into connected mode
  m_asSapProvider->Connect ();
}

void
EpcUeNas::DoDisconnect ()
{
  NS_LOG_FUNCTION (this);
  // remove tfts
  while (m_bidCounter > 0)
    {
      m_tftClassifier.Delete (m_bidCounter);
      m_bidCounter--;
    }
  this->Disconnect ();
  m_bearersToBeActivatedList = m_bearersToBeActivatedListForReconnection; //restore the bearer list to be activated for the next RRC connection

}

void
EpcUeNas::Disconnect ()
{
  NS_LOG_FUNCTION (this);
  m_asSapProvider->Disconnect ();
  SwitchToState (OFF);
}


void
EpcUeNas::ActivateEpsBearer (EpsBearer bearer, Ptr<EpcTft> tft)
{
  NS_LOG_FUNCTION (this);
  switch (m_state)
    {
    case ACTIVE:
      NS_FATAL_ERROR ("the necessary NAS signaling to activate a bearer after the initial context has already been setup is not implemented");
      break;

    default:
      BearerToBeActivated btba;
      btba.bearer = bearer;
      btba.tft = tft;
      m_bearersToBeActivatedList.push_back (btba);
      m_bearersToBeActivatedListForReconnection.push_back (btba);
      break;
    }
}

bool
EpcUeNas::Send (Ptr<Packet> packet)
{
  NS_LOG_FUNCTION (this << packet);

  switch (m_state)
    {
    case ACTIVE:
      {
        uint32_t id = m_tftClassifier.Classify (packet, EpcTft::UPLINK);
        NS_ASSERT ((id & 0xFFFFFF00) == 0);
        uint8_t bid = (uint8_t) (id & 0x000000FF);
        if (bid == 0)
          {
            NS_LOG_INFO ("Unable to send data since bid is " << bid );
            return false;
          }
        else
          {
            NS_LOG_INFO ("Nas sends packet to rrc for bid " << bid );
            m_asSapProvider->SendData (packet, bid);
            return true;
          }
      }
      break;

    default:
      this->AddPacketToBuffer (packet); //uplink data arrival in idle mode
      return false;
      break;
    }
}

void
EpcUeNas::DoNotifyConnectionSuccessful ()
{
  NS_LOG_FUNCTION (this);

  SwitchToState (ACTIVE); // will eventually activate dedicated bearers
}

void
EpcUeNas::DoNotifyConnectionFailed ()
{
  NS_LOG_FUNCTION (this);
  DoNotifyConnectionReleased ();
  // commented to prevent endless loop of connect->failure when bad connection  
  // Simulator::ScheduleNow (&LteAsSapProvider::Connect, m_asSapProvider); // immediately retry the connection
}

void
EpcUeNas::DoRecvData (Ptr<Packet> packet)
{
  NS_LOG_FUNCTION (this << packet);
  m_forwardUpCallback (packet);
}

void
EpcUeNas::DoNotifyConnectionReleased ()
{
  NS_LOG_FUNCTION (this);
  SwitchToState (OFF);
  DoDiscardBufferedUlPackets ();
}

void
EpcUeNas::DoActivateEpsBearer (EpsBearer bearer, Ptr<EpcTft> tft)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT_MSG (m_bidCounter < 11, "cannot have more than 11 EPS bearers");
  uint8_t bid = ++m_bidCounter;
  m_tftClassifier.Add (tft, bid);
}

EpcUeNas::State
EpcUeNas::GetState () const
{
  NS_LOG_FUNCTION (this);
  return m_state;
}

void
EpcUeNas::SwitchToState (State newState)
{
  NS_LOG_FUNCTION (this << ToString (newState));
  State oldState = m_state;
  m_state = newState;
  NS_LOG_INFO ("IMSI " << m_imsi << " NAS " << ToString (oldState) << " --> " << ToString (newState));
  m_stateTransitionCallback (oldState, newState);

  // actions to be done when entering a new state:
  switch (m_state)
    {
    case ACTIVE:
      for (std::list<BearerToBeActivated>::iterator it = m_bearersToBeActivatedList.begin ();
           it != m_bearersToBeActivatedList.end ();
           m_bearersToBeActivatedList.erase (it++))
        {
          DoActivateEpsBearer (it->bearer, it->tft);
        }
      break;

    default:
      break;
    }

}

void
EpcUeNas::AddPacketToBuffer (Ptr<Packet> packet)
{
  NS_LOG_FUNCTION (this << packet);

  if (m_UlBufferSize + packet->GetSize () <= m_maxUlBufferSize)
    {
      if (m_state == ATTACHING)
        {
          NS_LOG_LOGIC ("Ul Buffer: New packet added");
          m_UlBuffer.push_back (packet);
          m_UlBufferSize += packet->GetSize ();
          NS_LOG_LOGIC ("NumOfPackets = " << m_UlBuffer.size ());
          NS_LOG_LOGIC ("UlBufferSize = " << m_UlBufferSize);
          return;
        }
      if (m_asSapProvider->IsUeCamped ())
        {
          this->Connect ();
          m_asSapProvider->SetUlDataPendingFlag (true);
          SwitchToState (ATTACHING);
          NS_LOG_LOGIC ("Ul Buffer: New packet added");
          m_UlBuffer.push_back (packet);
          m_UlBufferSize += packet->GetSize ();
          NS_LOG_LOGIC ("NumOfPackets = " << m_UlBuffer.size ());
          NS_LOG_LOGIC ("UlBufferSize = " << m_UlBufferSize);
        }
      else
        {
          NS_ABORT_MSG_IF (m_state != OFF, "NAS should not drop packets in " << ToString (m_state) << " state" );
          NS_LOG_WARN (this << " NAS OFF, discarding packet");
        }
    }
  else
    {
      // Discard Application packet
      NS_LOG_LOGIC ("UlBuffer is full. Application packet discarded");
      NS_LOG_LOGIC ("MaxUlBufferSize = " << m_maxUlBufferSize);
      NS_LOG_LOGIC ("UlBufferSize    = " << m_UlBufferSize);
      NS_LOG_LOGIC ("packet size     = " << packet->GetSize ());
    }
}

void
EpcUeNas::DoSendBufferedUlPackets ()
{
  NS_LOG_FUNCTION (this);

  while (!m_UlBuffer.empty ())
    {
      uint32_t id = m_tftClassifier.Classify (m_UlBuffer.front (), EpcTft::UPLINK);
      NS_LOG_INFO ("id: " << id << ", assert condition:" << (id & 0xFFFFFF00));
      NS_ASSERT ((id & 0xFFFFFF00) == 0);
      uint8_t bid = (uint8_t) (id & 0x000000FF);
      NS_LOG_INFO ("bid: " << bid);
      if (bid == 0)
        {
          //do nothing
        }
      else
        {
          m_asSapProvider->SendData (m_UlBuffer.front (), bid);

        }
      m_UlBuffer.pop_front ();
    }
}

void 
EpcUeNas::DoDiscardBufferedUlPackets ()
{
  NS_LOG_FUNCTION (this);
  m_UlBuffer.clear();
  m_UlBufferSize = 0;
}

} // namespace ns3

