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
 * Author: Marco Miozzo  <marco.miozzo@cttc.es>
 *         Nicola Baldo  <nbaldo@cttc.es>
 * Modified by:
 *          Danilo Abrignani <danilo.abrignani@unibo.it> (Carrier Aggregation - GSoC 2015)
 *          Biljana Bojovic <biljana.bojovic@cttc.es> (Carrier Aggregation)
 *
 * Modified by Michele Polese <michele.polese@gmail.com>
 *    (support for RACH realistic model)
 *
 * Modified by Vignesh Babu <ns3-dev@esk.fraunhofer.de>
 *    (support for uplink synchronization;
 *    integrated the RACH realistic model and RRC_CONNECTED->RRC_IDLE
 *    state transition (taken from Lena-plus(work of Michele Polese)) and also enhanced both the modules)
 */

#ifndef LTE_ENB_MAC_H
#define LTE_ENB_MAC_H


#include <map>
#include <vector>
#include <ns3/lte-common.h>
#include <ns3/lte-mac-sap.h>
#include <ns3/lte-enb-cmac-sap.h>
#include <ns3/ff-mac-csched-sap.h>
#include <ns3/ff-mac-sched-sap.h>
#include <ns3/lte-enb-phy-sap.h>
#include "ns3/traced-value.h"
#include "ns3/trace-source-accessor.h"
#include <ns3/packet.h>
#include <ns3/packet-burst.h>
#include <ns3/lte-ccm-mac-sap.h>
#include "ns3/vector.h"
#include <ns3/mobility-model.h>

namespace ns3 {

class DlCqiLteControlMessage;
class UlCqiLteControlMessage;
class PdcchMapLteControlMessage;

/// DlHarqProcessesBuffer_t typedef
typedef std::vector <std::vector < Ptr<PacketBurst> > > DlHarqProcessesBuffer_t;
typedef std::multimap<uint8_t, Vector> RapIdPositionMap_t;

///Callback to fetch the Mobility model of the UE
typedef Callback< Ptr<MobilityModel>, uint64_t> LteUeMobilityModelCallback;

/**
 * This class implements the MAC layer of the eNodeB device
 */
class LteEnbMac :   public Object
{
  /// allow EnbMacMemberLteEnbCmacSapProvider class friend access
  friend class EnbMacMemberLteEnbCmacSapProvider;
  /// allow EnbMacMemberLteMacSapProvider<LteEnbMac> class friend access
  friend class EnbMacMemberLteMacSapProvider<LteEnbMac>;
  /// allow EnbMacMemberFfMacSchedSapUser class friend access
  friend class EnbMacMemberFfMacSchedSapUser;
  /// allow EnbMacMemberFfMacCschedSapUser class friend access
  friend class EnbMacMemberFfMacCschedSapUser;
  /// allow EnbMacMemberLteEnbPhySapUser class friend access
  friend class EnbMacMemberLteEnbPhySapUser;
  /// allow MemberLteCcmMacSapProvider<LteEnbMac> class friend access
  friend class MemberLteCcmMacSapProvider<LteEnbMac>;

public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  LteEnbMac (void);
  virtual ~LteEnbMac (void);
  virtual void DoDispose (void);

  /**
   * \brief Set the component carrier ID
   * \param index the component carrier ID
   */
  void SetComponentCarrierId (uint8_t index);
  /**
   * \brief Set the scheduler SAP provider
   * \param s a pointer SAP provider of the FF packet scheduler
   */
  void SetFfMacSchedSapProvider (FfMacSchedSapProvider* s);
  /**
   * \brief Get the scheduler SAP user
   * \return a pointer to the SAP user of the scheduler
   */
  FfMacSchedSapUser* GetFfMacSchedSapUser (void);
  /**
   * \brief Set the control scheduler SAP provider
   * \param s a pointer to the control scheduler SAP provider
   */
  void SetFfMacCschedSapProvider (FfMacCschedSapProvider* s);
  /**
   * \brief Get the control scheduler SAP user
   * \return a pointer to the control scheduler SAP user
   */
  FfMacCschedSapUser* GetFfMacCschedSapUser (void);



  /**
   * \brief Set the MAC SAP user
   * \param s a pointer to the MAC SAP user
   */
  void SetLteMacSapUser (LteMacSapUser* s);
  /**
   * \brief Get the MAC SAP provider
   * \return a pointer to the SAP provider of the MAC
   */
  LteMacSapProvider* GetLteMacSapProvider (void);
  /**
   * \brief Set the control MAC SAP user
   * \param s a pointer to the control MAC SAP user
   */
  void SetLteEnbCmacSapUser (LteEnbCmacSapUser* s);
  /**
   * \brief Get the control MAC SAP provider
   * \return a pointer to the control MAC SAP provider
   */
  LteEnbCmacSapProvider* GetLteEnbCmacSapProvider (void);


  /**
  * \brief Get the eNB-PHY SAP User
  * \return a pointer to the SAP User of the PHY
  */
  LteEnbPhySapUser* GetLteEnbPhySapUser ();

  /**
  * \brief Set the PHY SAP Provider
  * \param s a pointer to the PHY SAP provider
  */
  void SetLteEnbPhySapProvider (LteEnbPhySapProvider* s);

  /**
  * \brief Get the eNB-ComponentCarrierManager SAP User
  * \return a pointer to the SAP User of the ComponentCarrierManager
  */
  LteCcmMacSapProvider* GetLteCcmMacSapProvider ();

  /**
  * \brief Set the ComponentCarrierManager SAP user
  * \param s a pointer to the ComponentCarrierManager provider
  */
  void SetLteCcmMacSapUser (LteCcmMacSapUser* s);

  /**
   * Set and Get for PRACH ideal or real,
   * used by LteHelper
   *
   */
  void SetPrachMode (bool ideal);
  bool GetPrachMode () const;

  /**
   * TracedCallback signature for DL scheduling events.
   *
   * \param [in] frame Frame number.
   * \param [in] subframe Subframe number.
   * \param [in] rnti The C-RNTI identifying the UE.
   * \param [in] mcs0 The MCS for transport block..
   * \param [in] tbs0Size
   * \param [in] mcs1 The MCS for transport block.
   * \param [in] tbs1Size
   * \param [in] component carrier id
   */
  typedef void (*DlSchedulingTracedCallback)
    (const uint32_t frame, const uint32_t subframe, const uint16_t rnti,
    const uint8_t mcs0, const uint16_t tbs0Size,
    const uint8_t mcs1, const uint16_t tbs1Size, const uint8_t ccId);

  /**
   *  TracedCallback signature for UL scheduling events.
   *
   * \param [in] frame Frame number.
   * \param [in] subframe Subframe number.
   * \param [in] rnti The C-RNTI identifying the UE.
   * \param [in] mcs  The MCS for transport block
   * \param [in] tbsSize
   */
  typedef void (*UlSchedulingTracedCallback)
    (const uint32_t frame, const uint32_t subframe, const uint16_t rnti,
    const uint8_t mcs, const uint16_t tbsSize);

  /**
   *  TracedCallback signature for rach preamble rx events.
   *
   * \param [in] RachPreambleLteControlMessage msg
   */
  typedef void (*RachPreambleReceivedTracedCallback)
    (const Ptr<RachPreambleLteControlMessage> msg);

private:
  /**
  * \brief Receive a DL CQI ideal control message
  * \param msg the DL CQI message
  */
  void ReceiveDlCqiLteControlMessage  (Ptr<DlCqiLteControlMessage> msg);

  /**
  * \brief Receive a DL CQI ideal control message
  * \param msg the DL CQI message
  */
  void DoReceiveLteControlMessage (Ptr<LteControlMessage> msg);

  /**
  * \brief Receive a CE element containing the buffer status report
  * \param bsr the BSR message
  */
  void ReceiveBsrMessage  (MacCeListElement_s bsr);

  /**
  * \brief UL CQI report
  * \param ulcqi FfMacSchedSapProvider::SchedUlCqiInfoReqParameters
  */
  void DoUlCqiReport (FfMacSchedSapProvider::SchedUlCqiInfoReqParameters ulcqi);



  // forwarded from LteEnbCmacSapProvider
  /**
  * \brief Configure MAC function
  * \param ulBandwidth the UL bandwidth
  * \param dlBandwidth the DL bandwidth
  */
  void DoConfigureMac (uint8_t ulBandwidth, uint8_t dlBandwidth);
  /**
  * \brief Add UE function
  * \param rnti the RNTI
  */
  void DoAddUe (uint16_t rnti);
  /**
  * \brief Remove UE function
  * \param rnti the RNTI
  */
  void DoRemoveUe (uint16_t rnti);
  /**
  * \brief Add LC function
  * \param lcinfo the LC info
  * \param msu the LTE MAC SAP user
  */
  void DoAddLc (LteEnbCmacSapProvider::LcInfo lcinfo, LteMacSapUser* msu);
  /**
  * \brief Reconfigure LC function
  * \param lcinfo the LC info
  */
  void DoReconfigureLc (LteEnbCmacSapProvider::LcInfo lcinfo);
  void DoReleaseLc (uint16_t rnti);
  /**
  * \brief Release LC function
  * \param rnti the RNTI
  * \param lcid the LCID
  */
  void DoReleaseLc (uint16_t  rnti, uint8_t lcid);
  /**
  * \brief UE Update configuration request function
  * \param params LteEnbCmacSapProvider::UeConfig
  */
  void DoUeUpdateConfigurationReq (LteEnbCmacSapProvider::UeConfig params);
  /**
  * \brief Get RACH configuration function
  * \returns LteEnbCmacSapProvider::RachConfig
  */
  LteEnbCmacSapProvider::RachConfig DoGetRachConfig ();
  /**
  * \brief Allocate NC RA preamble function
  * \param rnti the RNTI
  * \returns LteEnbCmacSapProvider::AllocateNcRaPreambleReturnValue
  */
  LteEnbCmacSapProvider::AllocateNcRaPreambleReturnValue DoAllocateNcRaPreamble (uint16_t rnti);

  // forwarded from LteMacSapProvider
  /**
  * \brief Transmit PDU function
  * \param params LteMacSapProvider::TransmitPduParameters
  */
  void DoTransmitPdu (LteMacSapProvider::TransmitPduParameters params);
  /**
  * \brief Report Buffer Status function
  * \param params LteMacSapProvider::ReportBufferStatusParameters
  */
  void DoReportBufferStatus (LteMacSapProvider::ReportBufferStatusParameters params);


  // forwarded from FfMacCchedSapUser
  /**
  * \brief CSched Cell Config configure function
  * \param params FfMacCschedSapUser::CschedCellConfigCnfParameters
  */
  void DoCschedCellConfigCnf (FfMacCschedSapUser::CschedCellConfigCnfParameters params);
  /**
  * \brief CSched UE Config configure function
  * \param params FfMacCschedSapUser::CschedUeConfigCnfParameters
  */
  void DoCschedUeConfigCnf (FfMacCschedSapUser::CschedUeConfigCnfParameters params);
  /**
  * \brief CSched LC Config configure function
  * \param params FfMacCschedSapUser::CschedLcConfigCnfParameters
  */
  void DoCschedLcConfigCnf (FfMacCschedSapUser::CschedLcConfigCnfParameters params);
  /**
  * \brief CSched LC Release configure function
  * \param params FfMacCschedSapUser::CschedLcReleaseCnfParameters
  */
  void DoCschedLcReleaseCnf (FfMacCschedSapUser::CschedLcReleaseCnfParameters params);
  /**
  * \brief CSched UE Release configure function
  * \param params FfMacCschedSapUser::CschedUeReleaseCnfParameters
  */
  void DoCschedUeReleaseCnf (FfMacCschedSapUser::CschedUeReleaseCnfParameters params);
  /**
  * \brief CSched UE Config Update Indication function
  * \param params FfMacCschedSapUser::CschedUeConfigUpdateIndParameters
  */
  void DoCschedUeConfigUpdateInd (FfMacCschedSapUser::CschedUeConfigUpdateIndParameters params);
  /**
  * \brief CSched Cell Config Update Indication function
  * \param params FfMacCschedSapUser::CschedCellConfigUpdateIndParameters
  */
  void DoCschedCellConfigUpdateInd (FfMacCschedSapUser::CschedCellConfigUpdateIndParameters params);

  // forwarded from FfMacSchedSapUser
  /**
  * \brief Sched DL Config Indication function
  * \param ind FfMacSchedSapUser::SchedDlConfigIndParameters
  */
  void DoSchedDlConfigInd (FfMacSchedSapUser::SchedDlConfigIndParameters ind);
  /**
  * \brief Sched UL Config Indication function
  * \param params FfMacSchedSapUser::SchedUlConfigIndParameters
  */
  void DoSchedUlConfigInd (FfMacSchedSapUser::SchedUlConfigIndParameters params);

  // forwarded from LteEnbPhySapUser
  /**
  * \brief Subrame Indication function
  * \param frameNo frame number
  * \param subframeNo subframe number
  */
  void DoSubframeIndication (uint32_t frameNo, uint32_t subframeNo);

  /*
  *
  *In lena only the rapId is sent during preamble transmission
  *Here, rapId with imsi and UE position is sent as ctrl msg during preamble transmission
  *for collision detection
  */
  void DoReceiveRachPreamble (Ptr<RachPreambleLteControlMessage> msg);

  // forwarded by LteCcmMacSapProvider
  /**
   * Report MAC CE to scheduler
   * \param bsr the BSR
   */
  void DoReportMacCeToScheduler (MacCeListElement_s bsr);

public:
  /**
   * legacy public for use the Phy callback
   * \param p packet
   */
  void DoReceivePhyPdu (Ptr<Packet> p);

  /**
   * Set the callback to fetch the mobility model of UE
   *
   */
  void  SetLteUeMobilityModelCallback (LteUeMobilityModelCallback c);


private:
  /**
  * \brief UL Info List ELements HARQ Feedback function
  * \param params UlInfoListElement_s
  */
  void DoUlInfoListElementHarqFeeback (UlInfoListElement_s params);
  /**
  * \brief DL Info List ELements HARQ Feedback function
  * \param params DlInfoListElement_s
  */
  void DoDlInfoListElementHarqFeeback (DlInfoListElement_s params);

  /**
   *
   *  \params the list with vectors representing UEpos - eNBpos
   */
  bool EvaluateCollisionProbability (std::list<Vector> pointList);

  /**
   * The timing advance for an UE is calculated whenever PUCCH/SRS
   * data is received from UE. The timing advance update is sent through
   * TAC ctrl msg whenever there is a change in the timing advance value or
   * on a per-need basis (data transfer ongoing). Also, for dynamic timer
   * configuration the duration of the time alignment timer is based on the
   * UE relative speed.
   *
   * Since CA causes the secondary cells to be installed at the same position
   * as the primary, only one TAC is enough and the TAG is currently not implemented.
   * TA calculation and transmission currently takes place only on the primary carrier.
   * So, only one time alignment timer (of primary carrier) exists at UE and eNodeB.
   *
   * \param rnti the RNTI of the UE whose timing advance calculation has to carried out
   *
   */
  void SendTimingAdvanceCommand (uint16_t rnti);

  /**
   * Add UE's IMSI to a map with RNTI as the key so that UE's mobility model
   * can be retrieved to perform timing advance calculation
   *
   *
   * \param rnti the RNTI of the UE
   * \param imsi the IMSI of the UE
   */
  void DoSetRntiImsiMap (uint16_t rnti, uint64_t imsi);

  /**
   * Create an UeInfo structure for each UE and set
   * the default values of various parameters related
   * to timing advance calculation and random access process
   * in connected mode. Add this structure to a map with
   * the RNTI of the UE as key
   *
   *
   * \param rnti the RNTI of the UE
   */
  void SetUeInfoMap (uint16_t rnti);

  /**
   * Method is executed when the time alignment timer expires.
   * The HARQ buffers are flushed, SRS/PUCCH resources are released
   * and configured uplink grants and downlink assignments are cleared.
   *
   *
   * \param rnti the RNTI of the UE
   */
  void TimeAlignmentTimeout (uint16_t rnti);

  /**
   * Start the parallel time alignment timer of the UE connected to the eNodeB
   * whenever RAR or TAC Ctrl msg is received by the UE.
   * This timer is started at the same time as the UE's time alignment timer
   * to maintain synchronization and ensure no assert messages
   * are triggered or error occurs if the timers time out at different times.
   *
   *
   * \param timeAlignmentTimer the duration of the time alignment timer
   * \param rnti the RNTI of the UE whose timer at the eNodeB has to be restarted
   */
  void DoStartTimeAlignmentTimer (Time timeAlignmentTimer, uint16_t rnti);

  /**
   * Restart the TxRX inactivity timer and the RRC inactivity timer
   * whenever DL/UL data transfer is detected for an UE
   *
   *
   * \param rnti the RNTI of the UE which is undergoing data transfer
   */
  void RestartInactivityTimers (uint16_t rnti);

  /**
   * Indicate to MAC, msg 4 (RRC connection setup or RRC connection reconfiguration)
   * is ready for transmission so that contention resolution MAC CE (CRI ctrl msg)
   * can be sent in the same sub frame as msg4
   *
   *
   * \param rnti the RNTI of the UE whose message 4 is ready for transmission
   */
  void DoSetMsg4Ready (uint16_t rnti);

  /**
   * Get the value of the time alignment timer based on the
   * relative speed of the UE (speed of UE at 2 different instant w.r.t the eNodeB)
   *
   *
   * \param ueSpeed the relative speed of the UE
   * \return the time duration of the time alignment timer
   */
  uint16_t GetDynamicTimerValue (double ueSpeed);

  /**
   *  Upon downlink data arrival when the UE is uplink out-of-sync,
   *  the PDCCCH order using DCI 1A format is sent to bring the UE back in-sync
   *  through the random access procedure. This PDCCH order signal informs the UE to
   *  perform the non-contention based random access procedure
   *
   *
   *  \param rnti the RNTI of the UE which has to be bought back in-sync
   */
  void SendPdcchOrder (uint16_t rnti);

  /**
   * Method executed upon TxRxInactivity Timer expiry.
   * Serves no purpose other than to indicate that the timer for the UE has expired.
   *
   *
   * \param rnti the RNTI of the UE whose timer expired
   */
  void TxRxInactivityTimeout (uint16_t rnti);

  /**
   * Method executed when the time alignment timer margin area is entered
   * upon expiry of enterTimeAlignmentTimerMargin timer.
   * Timing advance calculation is carried out and TAC is sent if the
   * TxRxInactivity Timer is running and there is a change in the timing advance value
   *
   *
   * \param rnti the RNTI of the UE whose timer expired and TAC may have to be sent
   */
  void EnterTimeAlignmentTimerMargin (uint16_t rnti);

  LteUeMobilityModelCallback m_lteUeMobilityModelCallback; ///< callback to fetch UE mobility model

  std::map <uint16_t, uint64_t> m_rntiImsiMap; ///< Map of UEs RNTI and IMSI

  /**
   * Structure for each UE containing parameters related to
   * timing advance calculation and random access process
   * in connected mode.
   *
   */
  struct UeInfo
  {
    int currentTimingAdvance; ///<current timing advance value for the UE
    int lastTimingAdvance; ///<last(previous) timing advance value for the UE
    bool isRandomAccessResponse; ///<Set if the TAC is sent in random access response
    Time timeAlignmentTimer; ///<the duration of the time alignment timer
    /**
     * Time limit before the time alignment timer expires.
     * Upon expiry, the method LteEnbMac::TimeAlignmentTimeout is called.
     * Restarted whenever TAC is received by UE
     */
    EventId timeAlignmentTimeout;
    bool isUplinkOutOfSync; //set if the time alignment timer expire and UE goes uplink out-of-sync
    bool isNonContentionBasedRandomAccess; ///<set if RandomAccess is NonContentionBased, otherwise reset
    bool isMsg4Ready; ///<set if msg 4 is ready for transmission so contention resolution can be sent at the same time
    /**
     * Sliding window to determine to send TAC msg if
     * DL/UL data transfer for an UE occurs. Restarted for every
     * DL/UL data PDU transfer to/from an UE. Required to keep
     * the UE uplink time aligned during data transfer.
     * If expired, method LteEnbMac::TxRxInactivityTimeout is called.
     * TAC msg is only sent if this timer is running i.e data transfer ongoing
     */
    EventId txRxInactivityTimeout;
    /**
     * The event time duration = time alignment timer - Margin area
     * Upon its expiry, the method LteEnbMac::EnterTimeAlignmentTimerMargin
     * is called. This indicates that the Margin area (Time duration
     * just before the time alignment timer (TAT) expires) is entered
     * and if the txRxInactivity timer is running, TAC msg is sent to prevent the
     * expiry of the time alignment timer.
     * Value is 20% of TAT. Range: 0-2560ms
     */
    EventId enterTimeAlignmentTimerMargin;
    Time timestampOfLastTimingAdvance; ///<the time instance at which the last timing advance msg was sent for an UE
    double currentDistance; ///<the current distance of the UE from eNodeB
    double previousDistance; ///<the previous distance of the UE from eNodeB (corresponds to the last time when TAC msg was sent)
    bool isTimeAlignmentTimerMargin; ///<Set if the Margin area before the time alignment timer expires is entered
    bool rachStarted; ///<true, if RACH is started.
  };

  std::map<uint16_t, UeInfo> m_ueInfoMap; ///<map of UeInfo for each UE with RNTI as key

  bool m_dynamicTimerConfigured; ///<Set if dynamic timer configuration for time alignment timer is selected

  /**
   * Duration over which the active status (data transfer ongoing)
   * of the UE is checked for sending the timing advance updates
   *
   */
  Time m_txRxInactivityTimeoutDuration;

  /// rnti, lcid, SAP of the RLC instance
  std::map <uint16_t, std::map<uint8_t, LteMacSapUser*> > m_rlcAttached;

  std::vector <CqiListElement_s> m_dlCqiReceived; ///< DL-CQI received
  std::vector <FfMacSchedSapProvider::SchedUlCqiInfoReqParameters> m_ulCqiReceived; ///< UL-CQI received
  std::vector <MacCeListElement_s> m_ulCeReceived; ///< CE received (BSR up to now)

  std::vector <DlInfoListElement_s> m_dlInfoListReceived; ///< DL HARQ feedback received

  std::vector <UlInfoListElement_s> m_ulInfoListReceived; ///< UL HARQ feedback received

  bool m_message3Received;
  /*
  * Map of UE's info element (see 4.3.12 of FF MAC Scheduler API)
  */
  //std::map <uint16_t,UlInfoListElement_s> m_ulInfoListElements;



  LteMacSapProvider* m_macSapProvider; ///< the MAC SAP provider
  LteEnbCmacSapProvider* m_cmacSapProvider; ///< the CMAC SAP provider
  LteMacSapUser* m_macSapUser; ///< the MAC SAP user
  LteEnbCmacSapUser* m_cmacSapUser; ///< the CMAC SAP user


  FfMacSchedSapProvider* m_schedSapProvider; ///< the Sched SAP provider
  FfMacCschedSapProvider* m_cschedSapProvider; ///< the Csched SAP provider
  FfMacSchedSapUser* m_schedSapUser; ///< the Sched SAP user
  FfMacCschedSapUser* m_cschedSapUser; ///< the CSched SAP user

  // PHY-SAP
  LteEnbPhySapProvider* m_enbPhySapProvider; ///< the ENB Phy SAP provider
  LteEnbPhySapUser* m_enbPhySapUser; ///< the ENB Phy SAP user

  // Sap For ComponentCarrierManager 'Uplink case'
  LteCcmMacSapProvider* m_ccmMacSapProvider; ///< CCM MAC SAP provider
  LteCcmMacSapUser* m_ccmMacSapUser; ///< CCM MAC SAP user
  /**
   * frame number of current subframe indication
   */
  uint32_t m_frameNo;
  /**
   * subframe number of current subframe indication
   */
  uint32_t m_subframeNo;
  /**
   * Trace information regarding DL scheduling
   * Frame number, Subframe number, RNTI, MCS of TB1, size of TB1,
   * MCS of TB2 (0 if not present), size of TB2 (0 if not present)
   */
  TracedCallback<DlSchedulingCallbackInfo> m_dlScheduling;

  /**
   * Trace information regarding UL scheduling
   * Frame number, Subframe number, RNTI, MCS of TB, size of TB, component carrier id
   */
  TracedCallback<uint32_t, uint32_t, uint16_t,
                 uint8_t, uint16_t, uint8_t> m_ulScheduling;

  /**
   * Trace information regarding rach preamble rx
   *
   */
  TracedCallback<Ptr<RachPreambleLteControlMessage> > m_receiveRachPreambleTrace;

  uint8_t m_macChTtiDelay; ///< delay of MAC, PHY and channel in terms of TTIs


  std::map <uint16_t, DlHarqProcessesBuffer_t> m_miDlHarqProcessesPackets; ///< Packet under transmission of the DL HARQ process

  uint8_t m_numberOfRaPreambles; ///< number of RA preambles
  uint8_t m_preambleTransMax; ///< preamble transmit maximum
  uint8_t m_raResponseWindowSize; ///< RA response window size


  uint8_t m_pRachConfigurationIndex;
  uint8_t m_powerRampingStep;
  uint8_t m_preambleInitialReceivedTargetPower;
  uint8_t m_contentionResolutionTimer;
  uint8_t m_backoffIndicator;
  bool m_realPrach;

  /**
   * info associated with a preamble allocated for non-contention based RA
   *
   */
  struct NcRaPreambleInfo
  {
    uint16_t rnti; ///< rnti previously allocated for this non-contention based RA procedure
    Time expiryTime; ///< value the expiration time of this allocation (so that stale preambles can be reused)
  };

  /**
   * map storing as key the random access preamble IDs allocated for
   * non-contention based access, and as value the associated info
   *
   */
  std::map<uint8_t, NcRaPreambleInfo> m_allocatedNcRaPreambleMap;

  // stores rapId and node position (internal hack to evaluate if there can be a collision at the preamble)
  RapIdPositionMap_t m_receivedRachPreambleCount;

  std::map<uint16_t, uint32_t> m_rapIdRntiMap; ///< RAPID RNTI map, key changed from uint8_t to uint_16t as the key is RNTI (16 bit value)

  /// component carrier Id used to address sap
  uint8_t m_componentCarrierId;

};

} // end namespace ns3

#endif /* LTE_ENB_MAC_ENTITY_H */
