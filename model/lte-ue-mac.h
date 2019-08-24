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
 * Author: Nicola Baldo  <nbaldo@cttc.es>
 * Author: Marco Miozzo <mmiozzo@cttc.es>
 *
 * Modified by Michele Polese <michele.polese@gmail.com>
 *    (support for RACH realistic model)
 *
 * Modified by Vignesh Babu <ns3-dev@esk.fraunhofer.de>
 *    (support for uplink synchronization;
 *    integrated the RACH realistic model and RRC_CONNECTED->RRC_IDLE
 *    state transition (taken from Lena-plus(work of Michele Polese)) and also enhanced both the modules)
 */

#ifndef LTE_UE_MAC_ENTITY_H
#define LTE_UE_MAC_ENTITY_H



#include <map>

#include <ns3/lte-mac-sap.h>
#include <ns3/lte-ue-cmac-sap.h>
#include <ns3/lte-ue-phy-sap.h>
#include <ns3/nstime.h>
#include <ns3/event-id.h>
#include <vector>
#include <ns3/packet.h>
#include <ns3/packet-burst.h>
#include <ns3/traced-callback.h>


namespace ns3 {

class UniformRandomVariable;

class LteUeMac :   public Object
{
  /// allow UeMemberLteUeCmacSapProvider class friend access
  friend class UeMemberLteUeCmacSapProvider;
  /// allow UeMemberLteMacSapProvider class friend access
  friend class UeMemberLteMacSapProvider;
  /// allow UeMemberLteUePhySapUser class friend access
  friend class UeMemberLteUePhySapUser;

public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  LteUeMac ();
  virtual ~LteUeMac ();
  virtual void DoDispose (void);

  /**
  * \brief Get the LTE MAC SAP provider
  * \return a pointer to the LTE MAC SAP provider
  */
  LteMacSapProvider*  GetLteMacSapProvider (void);
  /**
  * \brief Set the LTE UE CMAC SAP user
  * \param s the LTE UE CMAC SAP User
  */
  void  SetLteUeCmacSapUser (LteUeCmacSapUser* s);
  /**
  * \brief Get the LTE CMAC SAP provider
  * \return a pointer to the LTE CMAC SAP provider
  */
  LteUeCmacSapProvider*  GetLteUeCmacSapProvider (void);

  /**
  * \brief Set the component carried ID
  * \param index the component carrier ID
  */
  void SetComponentCarrierId (uint8_t index);

  /**
  * \brief Get the PHY SAP user
  * \return a pointer to the SAP user of the PHY
  */
  LteUePhySapUser* GetLteUePhySapUser ();

  /**
  * \brief Set the PHY SAP Provider
  * \param s a pointer to the PHY SAP Provider
  */
  void SetLteUePhySapProvider (LteUePhySapProvider* s);

  /**
  * \brief Forwarded from LteUePhySapUser: trigger the start from a new frame
  *
  * \param frameNo frame number
  * \param subframeNo subframe number
  */
  void DoSubframeIndication (uint32_t frameNo, uint32_t subframeNo);

  /**
   * Assign a fixed random variable stream number to the random variables
   * used by this model.  Return the number of streams (possibly zero) that
   * have been assigned.
   *
   * \param stream first stream index to use
   * \return the number of stream indices assigned by this model
   */
  int64_t AssignStreams (int64_t stream);

  /**
   *
   * \param imsi the unique UE identifier
   */
  void SetImsi (uint64_t imsi);

  /**
   *
   * \return imsi the unique UE identifier
   */
  uint64_t GetImsi (void) const;


  /**
  * This method is used by the LteUeMac to notify the RaStatsCalculator about
  * the tx of a new RACH preamble.
  *
  */
  typedef Callback< void, uint64_t > RachPreambleTxTracedCallback;


  /**
  * This method is used by the LteUeMac to notify the RaStatsCalculator that
  * a new msg3 is queued for transmission
  *
  */
  typedef Callback< void, uint64_t > Msg3TxTracedCallback;

  /**
   * Used when timing advance is received
   *
   */
  typedef Callback< uint64_t, uint16_t, Time, std::string> TimeAlignmentTimerTracedCallback;

  /**
   * Used when RACH in RRC CONNECTED state is started
   *
   */
  typedef Callback< uint64_t, uint16_t, std::string> ConnectedRachStartTracedCallback;

  /**
   * This Method is called when the time alignment timer expires
   * and UE loses its uplink synchronization. It inform the RRC about
   * the loss of uplink alignment. Based on the response
   * from RRC, subsequent deletion process is performed to transition
   * the UE to an out of sync state (No deletion during handover).
   * The random access procedure is reset, the uplink BSR are cleared
   * and HARQ buffers are flushed.
   * (See 3GPP 36.321 section 5.2)
   *
   */
  void TimeAlignmentTimeout ();

private:
  // forwarded from MAC SAP
  /**
   * Transmit PDU function
   *
   * \param params LteMacSapProvider::TransmitPduParameters
   */
  void DoTransmitPdu (LteMacSapProvider::TransmitPduParameters params);
  /**
   * Report buffers status function
   *
   * \param params LteMacSapProvider::ReportBufferStatusParameters
   */
  void DoReportBufferStatus (LteMacSapProvider::ReportBufferStatusParameters params);

  // forwarded from UE CMAC SAP
  /**
   * Configure RACH function
   *
   * \param rc LteUeCmacSapProvider::RachConfig
   */
  void DoConfigureRach (LteUeCmacSapProvider::RachConfig rc);
  /**
   * Start contention based random access procedure function
   */
  void DoStartContentionBasedRandomAccessProcedure ();
  /**
   * Set RNTI
   *
   * \param rnti the RNTI
   */
  void DoSetRnti (uint16_t rnti);
  /**
   * Start non contention based random access procedure function
   *
   * \param rnti the RNTI
   * \param rapId the RAPID
   * \param prachMask the PRACH mask
   */
  void DoStartNonContentionBasedRandomAccessProcedure (uint16_t rnti, uint8_t rapId, uint8_t prachMask);
  /**
   * Add LC function
   *
   * \param lcId the LCID
   * \param lcConfig the logical channel config
   * \param msu the MSU
   */
  void DoAddLc (uint8_t lcId, LteUeCmacSapProvider::LogicalChannelConfig lcConfig, LteMacSapUser* msu);
  /**
   * Remove LC function
   *
   * \param lcId the LCID
   */
  void DoRemoveLc (uint8_t lcId);
  /// Reset function
  void DoReset ();
  void DoNotifyConnectionExpired ();

  /**
   * Receive notification from RRC that RACH is successful
   *
   */
  void DoNotifyConnectionSuccessful ();

  // forwarded from PHY SAP
  /**
   * Receive Phy PDU function
   *
   * \param p the packet
   */
  void DoReceivePhyPdu (Ptr<Packet> p);
  /**
   * Receive LTE control message function
   *
   * \param msg the LTE control message
   */
  void DoReceiveLteControlMessage (Ptr<LteControlMessage> msg);

  // internal methods
  /// Randomly sleect and send RA preamble function
  void RandomlySelectAndSendRaPreamble ();
  /**
   * Send RA preamble function
   *
   * \param contention if true randomly select and send the RA preamble
   */
  void SendRaPreamble (bool contention);
  /// Start waiting for RA response function
  void StartWaitingForRaResponse ();
  /**
   * Receive the RA response function
   *
   * \param raResponse RA response received
   */
  void RecvRaResponse (BuildRarListElement_s raResponse);
  /**
   * RA response timeout function
   *
   * \param contention if true randomly select and send the RA preamble
   */
  void RaResponseTimeout (bool contention);
  /// Send report buffer status
  void SendReportBufferStatus (void);
  /// Refresh HARQ processes packet buffer function
  void RefreshHarqProcessesPacketBuffer (void);
  void ContentionResolutionTimeout (void);

  /**
   * Set the value of Time alignment timer.
   * Inital value received in SIB2, subsequent values in
   * TAC MAC CE (TacLteControlMessage)
   *
   */
  void DoSetTimeAlignmentTimer (uint16_t timeAlignmentTimerCommon);

  /// component carrier Id --> used to address sap
  uint8_t m_componentCarrierId;

private:
  /// LcInfo structure
  struct LcInfo
  {
    LteUeCmacSapProvider::LogicalChannelConfig lcConfig; ///< logical channel config
    LteMacSapUser* macSapUser; ///< MAC SAP user
  };

  std::map <uint8_t, LcInfo> m_lcInfoMap; ///< logical channel info map

  LteMacSapProvider* m_macSapProvider; ///< MAC SAP provider

  LteUeCmacSapUser* m_cmacSapUser; ///< CMAC SAP user
  LteUeCmacSapProvider* m_cmacSapProvider; ///< CMAC SAP provider

  LteUePhySapProvider* m_uePhySapProvider; ///< UE Phy SAP provider
  LteUePhySapUser* m_uePhySapUser; ///< UE Phy SAP user

  std::map <uint8_t, LteMacSapProvider::ReportBufferStatusParameters> m_ulBsrReceived; ///< BSR received from RLC (the last one)


  Time m_bsrPeriodicity; ///< BSR periodicity
  Time m_bsrLast; ///< BSR last
  Time m_raPreambleStartTime;

  bool m_freshUlBsr; ///< true when a BSR has been received in the last TTI

  uint8_t m_harqProcessId; ///< HARQ process ID
  std::vector < Ptr<PacketBurst> > m_miUlHarqProcessesPacket; ///< Packets under transmission of the UL HARQ processes
  std::vector < uint8_t > m_miUlHarqProcessesPacketTimer; ///< timer for packet life in the buffer

  uint16_t m_rnti; ///< RNTI
  uint64_t  m_imsi;

  bool m_rachConfigured; ///< is RACH configured?


  bool m_rachPreambleReady;
  bool m_message3Ready;
  bool m_msg3BufferExpired; //variable not used
  bool DoMsg3Ready (void);
  void DoUpdateRaRnti (uint32_t raRnti);

  LteUeCmacSapProvider::RachConfig m_rachConfig; ///< RACH configuration
  uint8_t m_raPreambleId; ///< RA preamble ID
  uint8_t m_preambleTransmissionCounter; ///< preamble tranamission counter
  uint16_t m_backoffParameter; ///< backoff parameter
  EventId m_noRaResponseReceivedEvent; ///< no RA response received event ID
  EventId m_reSendRaPreambleEvent;
  EventId m_contentionResolutionTimeout;
  Ptr<UniformRandomVariable> m_raPreambleUniformVariable; ///< RA preamble random variable
  Ptr<UniformRandomVariable> m_backoffTime;

  uint32_t m_frameNo; ///< frame number
  uint32_t m_subframeNo; ///< subframe number
  uint8_t m_raRnti; ///< RA RNTI
  bool m_waitingForRaResponse; ///< waiting for RA response

  //uplink synchronzation parameters
  Time m_timeAlignmentTimer;  ///< the duration of the time alignment timer
  EventId m_timeAlignmentTimeoutEvent;  ///< After the time alignment timer expires, the event is executed and UE goes out of sync
  bool m_uplinkOutOfSync; ///< true, if the UE is out-of-sync
  bool m_contention; ///<to identify if Random Access in Connected mode is UE or eNodeB initiated
  bool m_rachStarted; ///<to indicate a Random Access procedure is already ongoing
  EventId m_waitforRarEvent; ///<When event expires start waiting for RAR reception

  /**
   * Send the C-RNTI MAC CE during contention based random
   * access in connected mode for UL data arrival.
   *
   */
  void SendCRntiMacCe (uint16_t tempRnti);

  TracedCallback< uint64_t> m_preambleTxStartTrace;
  TracedCallback< uint64_t > m_msg3TxStartTrace;

  /**
   * The 'ConnectedOutOfSnycRach' trace source. Fired when Random Access
   * occurs in RRC CONNECTED state.
   *
   */
  TracedCallback< uint64_t, uint16_t, std::string> m_ConnectedRachStartTrace;

  /**
    * The 'TimeAlignmentTimerUpdate' trace source. Fired when UE receives
    * timing advance in RAR or in TacLteControlMessage
    *
    */
  TracedCallback< uint64_t, uint16_t, Time, std::string> m_timeAlignmentTimerUpdateTrace;
};

} // namespace ns3

#endif // LTE_UE_MAC_ENTITY
