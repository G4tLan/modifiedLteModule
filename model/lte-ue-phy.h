/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010 TELEMATICS LAB, DEE - Politecnico di Bari
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
 * Author: Giuseppe Piro  <g.piro@poliba.it>
 * Author: Marco Miozzo <mmiozzo@cttc.es>
 *
 * Modified by Michele Polese <michele.polese@gmail.com>
 *    (support for RACH realistic model)
 *
 * Modified by Vignesh Babu <ns3-dev@esk.fraunhofer.de>
 *    (support for Paging, Radio Link Failure, uplink synchronization;
 *    integrated the RACH realistic model and RRC_CONNECTED->RRC_IDLE
 *    state transition (taken from Lena-plus(work of Michele Polese)) and also enhanced both the modules)
 */

#ifndef LTE_UE_PHY_H
#define LTE_UE_PHY_H


#include <ns3/lte-phy.h>
#include <ns3/ff-mac-common.h>
#include <ns3/lte-ue-cmac-sap.h>

#include <ns3/lte-control-messages.h>
#include <ns3/lte-amc.h>
#include <ns3/lte-ue-phy-sap.h>
#include <ns3/lte-ue-cphy-sap.h>
#include <ns3/ptr.h>
#include <ns3/lte-amc.h>
#include <set>
#include <ns3/lte-ue-power-control.h>


namespace ns3 {

class PacketBurst;
class LteEnbPhy;
class LteHarqPhy;


/**
 * \ingroup lte
 *
 * The LteSpectrumPhy models the physical layer of LTE
 */
class LteUePhy : public LtePhy
{

  /// allow UeMemberLteUePhySapProvider class friend access
  friend class UeMemberLteUePhySapProvider;
  /// allow MemberLteUeCphySapProvider<LteUePhy> class friend access
  friend class MemberLteUeCphySapProvider<LteUePhy>;

public:
  /**
   * \brief The states of the UE PHY entity
   */
  enum State
  {
    CELL_SEARCH = 0,
    SYNCHRONIZED,
    NUM_STATES
  };

  /**
   * @warning the default constructor should not be used
   */
  LteUePhy ();

  /**
   *
   * \param dlPhy the downlink LteSpectrumPhy instance
   * \param ulPhy the uplink LteSpectrumPhy instance
   */
  LteUePhy (Ptr<LteSpectrumPhy> dlPhy, Ptr<LteSpectrumPhy> ulPhy);

  virtual ~LteUePhy ();

  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  // inherited from Object
  virtual void DoInitialize (void);
  virtual void DoDispose (void);

  /**
   * \brief Get the PHY SAP provider
   * \return a pointer to the SAP Provider
   */
  LteUePhySapProvider* GetLteUePhySapProvider ();

  /**
  * \brief Set the PHY SAP User
  * \param s a pointer to the SAP user
  */
  void SetLteUePhySapUser (LteUePhySapUser* s);

  /**
   * \brief Get the CPHY SAP provider
   * \return a pointer to the SAP Provider
   */
  LteUeCphySapProvider* GetLteUeCphySapProvider ();

  /**
  * \brief Set the CPHY SAP User
  * \param s a pointer to the SAP user
  */
  void SetLteUeCphySapUser (LteUeCphySapUser* s);


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
   * Set and Get for PRACH ideal or real,
   * used by LteHelper
   *
   */
  void SetPrachMode (bool ideal);
  bool GetPrachMode () const;


  /**
   * \param pow the transmission power in dBm
   */
  void SetTxPower (double pow);

  /**
   * \return the transmission power in dBm
   */
  double GetTxPower () const;

  /**
   * \return ptr to UE Uplink Power Control entity
   */
  Ptr<LteUePowerControl> GetUplinkPowerControl () const;

  /**
   * \param nf the noise figure in dB
   */
  void SetNoiseFigure (double nf);

  /**
   * \return the noise figure in dB
   */
  double GetNoiseFigure () const;

  /**
   * \returns the TTI delay between MAC and channel
   */
  uint8_t GetMacChDelay (void) const;

  /**
   * \return a pointer to the LteSpectrumPhy instance relative to the downlink
   */
  Ptr<LteSpectrumPhy> GetDlSpectrumPhy () const;

  /**
   * \return a pointer to the LteSpectrumPhy instance relative to the uplink
   */
  Ptr<LteSpectrumPhy> GetUlSpectrumPhy () const;

  /**
   * \brief Create the PSD for the TX
   * \return the pointer to the PSD
   */
  virtual Ptr<SpectrumValue> CreateTxPowerSpectralDensity ();

  /**
   * \brief Set a list of sub channels to use in TX
   * \param mask a list of sub channels
   */
  void SetSubChannelsForTransmission (std::vector <int> mask);
  /**
   * \brief Get a list of sub channels to use in RX
   * \return a list of sub channels
   */
  std::vector <int> GetSubChannelsForTransmission (void);

  /**
   * \brief Get a list of sub channels to use in RX
   * \param mask list of sub channels
   */
  void SetSubChannelsForReception (std::vector <int> mask);
  /**
   * \brief Get a list of sub channels to use in RX
   * \return a list of sub channels
   */
  std::vector <int> GetSubChannelsForReception (void);

  /**
  * \brief Create the DL CQI feedback from SINR values perceived at
  * the physical layer with the signal received from eNB
  * \param sinr SINR values vector
  * \return a DL CQI control message containing the CQI feedback
  */
  Ptr<DlCqiLteControlMessage> CreateDlCqiFeedbackMessage (const SpectrumValue& sinr);



  // inherited from LtePhy
  virtual void GenerateCtrlCqiReport (const SpectrumValue& sinr);
  virtual void GenerateDataCqiReport (const SpectrumValue& sinr);
  /**
  * \brief Create the mixed CQI report
  *
  * \param sinr SINR values vector
  */
  virtual void GenerateMixedCqiReport (const SpectrumValue& sinr);
  virtual void ReportInterference (const SpectrumValue& interf);
  /**
  * \brief Create the mixed CQI report
  *
  * \param interf interference values vector
  */
  virtual void ReportDataInterference (const SpectrumValue& interf);
  virtual void ReportRsReceivedPower (const SpectrumValue& power);

  // callbacks for LteSpectrumPhy
  /**
  * \brief Receive LTE control message list function
  *
  * \param msgList LTE control message list
  */
  virtual void ReceiveLteControlMessageList (std::list<Ptr<LteControlMessage> > msgList);
  /**
  * \brief Receive PSS function
  *
  * \param cellId the cell ID
  * \param p PSS list
  */
  virtual void ReceivePss (uint16_t cellId, Ptr<SpectrumValue> p);


  /**
   * \brief PhySpectrum received a new PHY-PDU
   * \param p the packet received
   */
  void PhyPduReceived (Ptr<Packet> p);


  /**
  * \brief trigger from eNB the start from a new frame
  *
  * \param frameNo frame number
  * \param subframeNo subframe number
  */
  void SubframeIndication (uint32_t frameNo, uint32_t subframeNo);


  /**
   * \brief Send the SRS signal in the last symbols of the frame
   */
  void SendSrs ();

  /**
   * \brief PhySpectrum generated a new DL HARQ feedback
   * \param mes the DlInfoListElement_s
   */
  virtual void ReceiveLteDlHarqFeedback (DlInfoListElement_s mes);

  /**
   * \brief Set the HARQ PHY module
   * \param harq the HARQ PHY module
   */
  void SetHarqPhyModule (Ptr<LteHarqPhy> harq);

  /**
   * \return The current state
   */
  State GetState () const;

  /**
   * TracedCallback signature for state transition events.
   *
   * \param [in] cellId
   * \param [in] rnti
   * \param [in] oldState
   * \param [in] newState
   */
  typedef void (*StateTracedCallback)
    (uint16_t cellId, uint16_t rnti, State oldState, State newState);

  /**
   * TracedCallback signature for tx events.
   *
   *
   * \param [in] imsi
   * \param [in] cellId
   */
  typedef void (*StartTxTracedCallback)
    (const uint64_t imsi, const uint16_t cellId, const uint16_t rnti);

  /**
   * TracedCallback signature for cell RSRP and SINR report.
   *
   * \param [in] cellId
   * \param [in] rnti
   * \param [in] rsrp
   * \param [in] sinr
   * \param [in] componentCarrierId
   */
  typedef void (*RsrpSinrTracedCallback)
    (uint16_t cellId, uint16_t rnti,
    double rsrp, double sinr, uint8_t componentCarrierId);

  /**
   * TracedCallback signature for cell RSRP and RSRQ.
   *
   * \param [in] rnti
   * \param [in] cellId
   * \param [in] rsrp
   * \param [in] rsrq
   * \param [in] isServingCell
   * \param [in] componentCarrierId
   */
  typedef void (*RsrpRsrqTracedCallback)
    (uint16_t rnti, uint16_t cellId, double rsrp, double rsrq,
    bool isServingCell, uint8_t componentCarrierId);

  /**
   * Reset the UE PHY due to the timeAlignmentTimer expiry.
   * The HARQ buffers are flushed, SRS/PUCCH resources are released
   * and configured uplink grants and downlink assignments are cleared.
   * The UE is considered to be uplink out-of-sync
   *
   */
  void DoResetToOutOfSync ();

  /**
   * Reset the UE to the idle camped state by clearing the
   * appropriate parameters at the PHY
   *
   */
  void DoResetToCamped ();

  void Disconnect (void);

private:
  /**
   * Reset the number of preamble transmission attempts
   * to zero when random access is successfully completed.
   *
   */
  void DoNotifyConnectionSuccessful ();

  /**
   * Configure the parameters for Radio Link Failure detection
   * when RAR is received by UE. RAR reception indicates that the UE
   * is both uplink and downlink synchronized and radio link failure
   * detection procedure can be started.
   *
   */
  void DoConfigureRadioLinkFailureDetection ();

  /**
   * When T310 timer is started, it indicates that physical layer
   * problems are detected at the UE and the recovery process is
   * started by checking if the radio frames are in-sync for N311
   * consecutive times.
   *
   */
  void DoStartInSnycDetection ();

  /**
   * Radio link monitoring is started to detect downlink radio link
   * quality when the UE is both uplink and downlink synchronized
   * (UE in CONNECTED_NORMALLY state).
   * Upon detection of radio link failure, RRC connection is released
   * and the UE moves to idle state to camp on a suitable cell. The procedure is implemented
   * as per 3GPP TS 36.213 4.2.1 and TS 36.133 7.6. When the downlink
   * radio link quality estimated over the last 200 ms period becomes worse than
   * the threshold Qout, an out-of-sync indication is sent to RRC. When the
   * downlink radio link quality estimated over the last 100 ms period becomes
   * better than the threshold Qin, an in-sync indication is sent to RRC.
   */
  void RadioLinkFailureDetection (double sinr_dB);

  /**
   * Calculate the paging cycle and paging occasion for the UE
   * according to the paging configuration received in SIB2.
   * See 3GPP 36.304 section 7
   *
   */
  void DoConfigurePaging (LteRrcSap::PcchConfig pagingConfig);

  /**
   * Method executed when "no RLF evaluation period" ends.
   * Serves no purpose other than to indicate that the RLF evaluation 
   * for out-of-sync or in-sync indications can be resumed. 
   */
  void ResumeRlfEvaluation();

  //Parameters for paging and radio link failure----------------------
  bool m_downlinkInSync; ///< When set, DL SINR evaluation for out-of-sync indications is conducted.
  uint16_t m_qoutEvaluationPeriod; ///< the downlink radio link quality is estimated over this period for detecting out-of-syncs
  uint16_t m_qinEvaluationPeriod; ///< the downlink radio link quality is estimated over this period for detecting in-syncs
  uint16_t m_numOfSubframes; ///< count the number of subframes for which the downlink radio link quality is estimated
  uint16_t m_numOfFrames; ///< count the number of frames for which the downlink radio link quality is estimated
  Time m_noEvaluationPeriod; ///< time duration between evaluation of successive out-of-sync or in-sync indications
  /**
   * For RLF evaulation, 3GPP TS 36.133 7.6.2.1 states that
   * two successive (out-of-sync or in-sync) indications from Layer 1
   * shall be separated by at least 10 ms. This separation period is
   * given by the m_noEvaluationPeriod parameter during which no RLF 
   * detection takes place (After notifying RRC about a out-of-sync or 
   * in-sync indication, the DL SINR is evaluted for detecting the next
   * out-of-sync or in-sync indication, only after the time duration given 
   * by the m_noEvaluationPeriod parameter has elapsed). After this time 
   * duration has elapsed, this event is executed indicating that the RLF evaluation
   * for the next indication can be resumed. 
   */
  EventId m_noRlfEvaluation; 

  /**
   * The 'Qin' attribute.
   * corresponds to 2% block error rate of a hypothetical PDCCH transmission
   * taking into account the PCFICH errors with transmission parameters.
   */
  double m_qIn;

  /**
   * The 'Qout' attribute.
   * corresponds to 2% block error rate of a hypothetical PDCCH transmission
   * taking into account the PCFICH errors with transmission parameters.
   */
  double m_qOut;
  double m_sinr_dB_Frame; ///<the average sinr per radio frame
  bool m_uplinkInSync; ///< set when UE is uplink synchronized (i.e when it receives RAR)

  uint32_t m_pagingFrame; ///< the frame in which paging message will be received
  uint16_t m_pagingOccasion; ///< the sub frame in which paging message will be received
  uint16_t m_pagingCycle; ///< the paging cycle of the UE
  bool m_pagingConfigured; ///< set if paging configuration is received in SIB2
  bool m_isConnected; ///< set when the RACH is successful, initiates the radio link failure detection
  //------------------------------------------------

  bool EvaluatePrachSubframe (uint32_t frameNo, uint32_t subframeNo);
  /**
   * Set transmit mode 1 gain function
   *
   * \param [in] gain
   */
  void SetTxMode1Gain (double gain);
  /**
   * Set transmit mode 2 gain function
   *
   * \param [in] gain
   */
  void SetTxMode2Gain (double gain);
  /**
   * Set transmit mode 3 gain function
   *
   * \param [in] gain
   */
  void SetTxMode3Gain (double gain);
  /**
   * Set transmit mode 4 gain function
   *
   * \param [in] gain
   */
  void SetTxMode4Gain (double gain);
  /**
   * Set transmit mode 5 gain function
   *
   * \param [in] gain
   */
  void SetTxMode5Gain (double gain);
  /**
   * Set transmit mode 6 gain function
   *
   * \param [in] gain
   */
  void SetTxMode6Gain (double gain);
  /**
   * Set transmit mode 7 gain function
   *
   * \param [in] gain
   */
  void SetTxMode7Gain (double gain);
  /**
   * Set transmit mode gain function
   *
   * \param [in] txMode
   * \param [in] gain
   */
  void SetTxModeGain (uint8_t txMode, double gain);

  /**
   * queue subchannels for transmission function
   *
   * \param [in] rbMap
   */
  void QueueSubChannelsForTransmission (std::vector <int> rbMap);


  /**
   * internal method that takes care of generating CQI reports,
   * calculating the RSRP and RSRQ metrics, and generating RSRP+SINR traces
   *
   * \param sinr
   */
  void GenerateCqiRsrpRsrq (const SpectrumValue& sinr);


  /**
   * \brief Layer-1 filtering of RSRP and RSRQ measurements and reporting to
   *        the RRC entity.
   *
   * Initially executed at +0.200s, and then repeatedly executed with
   * periodicity as indicated by the *UeMeasurementsFilterPeriod* attribute.
   */
  void ReportUeMeasurements ();

  /**
   * Switch the UE PHY to the given state.
   * \param s the destination state
   */
  void SwitchToState (State s);

  // UE CPHY SAP methods
  /// Reset function
  void DoReset ();
  /**
   * Start the cell search function
   * \param dlEarfcn the DL EARFCN
   */
  void DoStartCellSearch (uint32_t dlEarfcn);
  /**
   * Synchronize with ENB function
   * \param cellId the cell ID
   */
  void DoSynchronizeWithEnb (uint16_t cellId);
  /**
   * Synchronize with ENB function
   * \param cellId the cell ID
   * \param dlEarfcn the DL EARFCN
   */
  void DoSynchronizeWithEnb (uint16_t cellId, uint32_t dlEarfcn);
  /**
   * Set DL bandwidth function
   * \param dlBandwidth the DL bandwidth
   */
  void DoSetDlBandwidth (uint8_t dlBandwidth);
  /**
   * Configure UL uplink function
   * \param ulEarfcn UL EARFCN
   * \param ulBandwidth the UL bandwidth
   */
  void DoConfigureUplink (uint32_t ulEarfcn, uint8_t ulBandwidth);
  /**
   * Configure reference signal power function
   * \param referenceSignalPower reference signal power
   */
  void DoConfigureReferenceSignalPower (int8_t referenceSignalPower);
  /**
   * Set RNTI function
   * \param rnti the RNTI
   */
  void DoSetRnti (uint16_t rnti);
  /**
   * Set transmission mode function
   * \param txMode the transmission mode
   */
  void DoSetTransmissionMode (uint8_t txMode);
  /**
   * Set SRS configuration index function
   * \param srcCi the SRS configuration index
   */
  void DoSetSrsConfigurationIndex (uint16_t srcCi);
  /**
   * Set PA function
   * \param pa the PA value
   */
  void DoSetPa (double pa);

  // UE PHY SAP methods
  virtual void DoSendMacPdu (Ptr<Packet> p);
  /**
   * Send LTE control message function
   * \param msg the LTE control message
   */
  virtual void DoSendLteControlMessage (Ptr<LteControlMessage> msg);
  virtual void DoSendRachPreamble (Ptr<RachPreambleLteControlMessage> msg);
  virtual void DoDeletePrachPreamble ();

  /// A list of sub channels to use in TX.
  std::vector <int> m_subChannelsForTransmission;
  /// A list of sub channels to use in RX.
  std::vector <int> m_subChannelsForReception;

  std::vector< std::vector <int> > m_subChannelsForTransmissionQueue; ///< subchannels for transmission queue


  bool m_realPrach;
  bool m_rachMessageReceivedFromMac;
  bool m_rachConfigured;
  int8_t m_powerRampingStep;
  int8_t m_preambleInitialReceivedTargetPower;
  int32_t m_preambleReceivedTargetPower;
  uint32_t m_numPrachTx;

  void DoConfigurePrach (LteUeCmacSapProvider::RachConfig rc);


  Ptr<LteAmc> m_amc; ///< AMC

  /**
   * The `EnableUplinkPowerControl` attribute. If true, Uplink Power Control
   * will be enabled.
   */
  bool m_enableUplinkPowerControl;
  /**
   * The `EnablePrachPowerControl` attribute. If true, Prach Power Control
   * will be enabled.
   *
   */
  bool m_enablePrachPowerControl;

  void EvaluatePreambleReceivedTargetPower ();

  /// Pointer to UE Uplink Power Control entity.
  Ptr<LteUePowerControl> m_powerControl;

  /// Wideband Periodic CQI. 2, 5, 10, 16, 20, 32, 40, 64, 80 or 160 ms.
  Time m_p10CqiPeriodicity;
  Time m_p10CqiLast; ///< last periodic CQI

  /**
   * SubBand Aperiodic CQI. Activated by DCI format 0 or Random Access Response
   * Grant.
   * \note Defines a periodicity for academic studies.
   */
  Time m_a30CqiPeriodicity;
  Time m_a30CqiLast; ///< last aperiodic CQI

  LteUePhySapProvider* m_uePhySapProvider; ///< UE Phy SAP provider
  LteUePhySapUser* m_uePhySapUser; ///< UE Phy SAP user

  LteUeCphySapProvider* m_ueCphySapProvider; ///< UE CPhy SAP provider
  LteUeCphySapUser* m_ueCphySapUser; ///< UE CPhy SAP user

  uint16_t  m_rnti; ///< the RNTI
  uint64_t  m_imsi;

  uint8_t m_transmissionMode; ///< the transmission mode
  std::vector <double> m_txModeGain; ///< the transmit mode gain

  uint16_t m_srsPeriodicity; ///< SRS periodicity
  uint16_t m_srsSubframeOffset; ///< SRS subframe offset
  uint16_t m_srsConfigured; ///< SRS configured
  Time     m_srsStartTime; ///< SRS start time

  double m_paLinear; ///< PA linear

  bool m_dlConfigured; ///< DL configured?
  bool m_ulConfigured; ///< UL configured?
  bool m_rarReceived;

  /**
   * Called from MAC when RAR is received
   *
   */
  void DoNotifyRarReceived (void);

  /**
   * Called when MAC contention resolution timer expires
   *
   */
  void DoNotifyConnectionExpired (void);

  /// The current UE PHY state.
  State m_state;
  /**
   * The `StateTransition` trace source. Fired upon every UE PHY state
   * transition. Exporting the serving cell ID, RNTI, old state, and new state.
   */
  TracedCallback<uint16_t, uint16_t, State, State> m_stateTransitionTrace;

  TracedCallback<uint64_t, uint16_t, uint16_t> m_prachTxStart;

  /// \todo Can be removed.
  uint8_t m_subframeNo;
  uint32_t m_frameNo;

  bool m_rsReceivedPowerUpdated; ///< RS receive power updated?
  SpectrumValue m_rsReceivedPower; ///< RS receive power

  bool m_rsInterferencePowerUpdated; ///< RS interference power updated?
  SpectrumValue m_rsInterferencePower; ///< RS interference power

  bool m_dataInterferencePowerUpdated; ///< data interference power updated?
  SpectrumValue m_dataInterferencePower; ///< data interference power

  bool m_pssReceived; ///< PSS received?
  /// PssElement structure
  struct PssElement
  {
    uint16_t cellId; ///< cell ID
    double pssPsdSum; ///< PSS PSD sum
    uint16_t nRB; ///< number of RB
  };
  std::list <PssElement> m_pssList; ///< PSS list

  /**
   * The `RsrqUeMeasThreshold` attribute. Receive threshold for PSS on RSRQ
   * in dB.
   */
  double m_pssReceptionThreshold;

  /// Summary results of measuring a specific cell. Used for layer-1 filtering.
  struct UeMeasurementsElement
  {
    double rsrpSum;   ///< Sum of RSRP sample values in linear unit.
    uint8_t rsrpNum;  ///< Number of RSRP samples.
    double rsrqSum;   ///< Sum of RSRQ sample values in linear unit.
    uint8_t rsrqNum;  ///< Number of RSRQ samples.
  };

  /**
   * Store measurement results during the last layer-1 filtering period.
   * Indexed by the cell ID where the measurements come from.
   */
  std::map <uint16_t, UeMeasurementsElement> m_ueMeasurementsMap;
  /**
   * The `UeMeasurementsFilterPeriod` attribute. Time period for reporting UE
   * measurements, i.e., the length of layer-1 filtering (default 200 ms).
   */
  Time m_ueMeasurementsFilterPeriod;
  /// \todo Can be removed.
  Time m_ueMeasurementsFilterLast;

  Ptr<LteHarqPhy> m_harqPhyModule; ///< HARQ phy module

  uint32_t m_raPreambleId; ///< RA preamble ID
  uint32_t m_raRnti; ///< RA rnti

  /**
   * The `ReportCurrentCellRsrpSinr` trace source. Trace information regarding
   * RSRP and average SINR (see TS 36.214). Exporting cell ID, RNTI, RSRP, and
   * SINR. Moreover it reports the m_componentCarrierId.
   */
  TracedCallback<uint16_t, uint16_t, double, double, uint8_t> m_reportCurrentCellRsrpSinrTrace;
  /**
   * The `RsrpSinrSamplePeriod` attribute. The sampling period for reporting
   * RSRP-SINR stats.
   */
  uint16_t m_rsrpSinrSamplePeriod;
  /**
   * The `RsrpSinrSampleCounter` attribute. The sampling counter for reporting
   * RSRP-SINR stats.
   */
  uint16_t m_rsrpSinrSampleCounter;

  /**
   * The `ReportUeMeasurements` trace source. Contains trace information
   * regarding RSRP and RSRQ measured from a specific cell (see TS 36.214).
   * Exporting RNTI, the ID of the measured cell, RSRP (in dBm), RSRQ (in dB),
   * and whether the cell is the serving cell. Moreover it report the m_componentCarrierId.
   */
  TracedCallback<uint16_t, uint16_t, double, double, bool, uint8_t> m_reportUeMeasurements;

  EventId m_sendSrsEvent; ///< send SRS event

  /**
   * The `UlPhyTransmission` trace source. Contains trace information regarding
   * PHY stats from UL Tx perspective. Exporting a structure with type
   * PhyTransmissionStatParameters.
   */
  TracedCallback<PhyTransmissionStatParameters> m_ulPhyTransmission;


  Ptr<SpectrumValue> m_noisePsd; ///< Noise power spectral density for
                                 ///the configured bandwidth

}; // end of `class LteUePhy`


}

#endif /* LTE_UE_PHY_H */
