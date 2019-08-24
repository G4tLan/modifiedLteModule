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
 *         Budiarto Herman <budiarto.herman@magister.fi>
 *
 * Modified by Michele Polese <michele.polese@gmail.com>
 *    (support for RACH realistic model and RRC_CONNECTED->RRC_IDLE state transition)
 *
 * Modified by Vignesh Babu <ns3-dev@esk.fraunhofer.de>
 *    (support for Paging, Cell Reselection, Radio Link Failure, Handover Failure, uplink synchronization;
 *    integrated the RACH realistic model and RRC_CONNECTED->RRC_IDLE
 *    state transition (taken from Lena-plus(work of Michele Polese)) and also enhanced both the modules)
 */

#ifndef LTE_UE_RRC_H
#define LTE_UE_RRC_H

#include <ns3/object.h>
#include <ns3/packet.h>
#include <ns3/lte-ue-cmac-sap.h>
#include <ns3/lte-pdcp-sap.h>
#include <ns3/lte-as-sap.h>
#include <ns3/lte-ue-cphy-sap.h>
#include <ns3/lte-rrc-sap.h>
#include <ns3/traced-callback.h>
#include "ns3/component-carrier-ue.h"
#include <ns3/lte-ue-ccm-rrc-sap.h>
#include <vector>

#include <map>
#include <set>

#define MIN_NO_CC 1
#define MAX_NO_CC 5 // this is the maximum number of carrier components allowed by 3GPP up to R13

namespace ns3 {


/**
 * \brief Artificial delay of UE measurements procedure.
 *
 * i.e. the period between the time layer-1-filtered measurements from PHY
 * layer is received and the earliest time the actual measurement report
 * submission to the serving cell is invoked.
 *
 * This delay exists because of racing condition between several UE measurements
 * functions which happen to be scheduled at the same time. The delay ensures
 * that:
 *  - measurements (e.g., layer-3 filtering) are always performed before
 *    reporting, thus the latter always use the latest measured RSRP and RSRQ;
 *    and
 *  - time-to-trigger check is always performed before the reporting, so there
 *    would still be chance for it to cancel the reporting if necessary.
 */
static const Time UE_MEASUREMENT_REPORT_DELAY = MicroSeconds (1);


class LteRlc;
class LteMacSapProvider;
class LteUeCmacSapUser;
class LteUeCmacSapProvider;
class LteDataRadioBearerInfo;
class LteSignalingRadioBearerInfo;

/**
 *
 *
 */
class LteUeRrc : public Object
{

  /// allow UeMemberLteUeCmacSapUser class friend access
  friend class UeMemberLteUeCmacSapUser;
  /// allow UeRrcMemberLteEnbCmacSapUser class friend access
  friend class UeRrcMemberLteEnbCmacSapUser;
  /// allow LtePdcpSpecificLtePdcpSapUser<LteUeRrc> class friend access
  friend class LtePdcpSpecificLtePdcpSapUser<LteUeRrc>;
  /// allow MemberLteAsSapProvider<LteUeRrc> class friend access
  friend class MemberLteAsSapProvider<LteUeRrc>;
  /// allow MemberLteUeCphySapUser<LteUeRrc> class friend access
  friend class MemberLteUeCphySapUser<LteUeRrc>;
  /// allow MemberLteUeRrcSapProvider<LteUeRrc> class friend access
  friend class MemberLteUeRrcSapProvider<LteUeRrc>;
  /// allow MemberLteUeCcmRrcSapUser<LteUeRrc> class friend access
  friend class MemberLteUeCcmRrcSapUser<LteUeRrc>;

public:
  /**
   * The states of the UE RRC entity
   *
   */
  enum State
  {
    IDLE_START = 0,
    IDLE_CELL_SEARCH,
    IDLE_WAIT_MIB_SIB1,
    IDLE_WAIT_MIB,
    IDLE_WAIT_SIB1,
    IDLE_CAMPED_NORMALLY,
    IDLE_WAIT_SIB2,
    IDLE_RANDOM_ACCESS,
    IDLE_CONNECTING,
    CONNECTED_NORMALLY,
    CONNECTED_HANDOVER,
    CONNECTED_PHY_PROBLEM,
    CONNECTED_REESTABLISHING,
    NUM_STATES
  };


  /**
   * create an RRC instance for use within an ue
   *
   */
  LteUeRrc ();



  /**
   * Destructor
   */
  virtual ~LteUeRrc ();


  // inherited from Object
private:
  virtual void DoInitialize (void);
  virtual void DoDispose (void);
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  /// Initiaize SAP
  void InitializeSap (void);

  /**
   * set the CPHY SAP this RRC should use to interact with the PHY
   *
   * \param s the CPHY SAP Provider
   */
  void SetLteUeCphySapProvider (LteUeCphySapProvider * s);
  /**
   * set the CPHY SAP this RRC should use to interact with the PHY
   *
   * \param s the CPHY SAP Provider
   * \param index the index
   */
  void SetLteUeCphySapProvider (LteUeCphySapProvider * s, uint8_t index);

  /**
   *
   *
   * \return s the CPHY SAP User interface offered to the PHY by this RRC
   */
  LteUeCphySapUser* GetLteUeCphySapUser ();
  /**
   *
   * \param index the index
   * \return s the CPHY SAP User interface offered to the PHY by this RRC
   */
  LteUeCphySapUser* GetLteUeCphySapUser (uint8_t index);

  /**
   * set the CMAC SAP this RRC should interact with
   * \brief This function is overloaded to maintain backward compatibility
   * \param s the CMAC SAP Provider to be used by this RRC
   */
  void SetLteUeCmacSapProvider (LteUeCmacSapProvider * s);
  /**
   * set the CMAC SAP this RRC should interact with
   * \brief This function is overloaded to maintain backward compatibility
   * \param s the CMAC SAP Provider to be used by this RRC
   * \param index the index
   */
  void SetLteUeCmacSapProvider (LteUeCmacSapProvider * s, uint8_t index);

  /**
   * \brief This function is overloaded to maintain backward compatibility
   * \return s the CMAC SAP User interface offered to the MAC by this RRC
   */
  LteUeCmacSapUser* GetLteUeCmacSapUser ();
  /**
   * \brief This function is overloaded to maintain backward compatibility
   * \param index the index
   * \return s the CMAC SAP User interface offered to the MAC by this RRC
   */
  LteUeCmacSapUser* GetLteUeCmacSapUser (uint8_t index);


  /**
   * set the RRC SAP this RRC should interact with
   *
   * \param s the RRC SAP User to be used by this RRC
   */
  void SetLteUeRrcSapUser (LteUeRrcSapUser * s);

  /**
   *
   *
   * \return s the RRC SAP Provider interface offered to the MAC by this RRC
   */
  LteUeRrcSapProvider* GetLteUeRrcSapProvider ();

  /**
   * set the MAC SAP provider. The ue RRC does not use this
   * directly, but it needs to provide it to newly created RLC instances.
   *
   * \param s the MAC SAP provider that will be used by all
   * newly created RLC instances
   */
  void SetLteMacSapProvider (LteMacSapProvider* s);

  /**
   * Set the AS SAP user to interact with the NAS entity
   *
   * \param s the AS SAP user
   */
  void SetAsSapUser (LteAsSapUser* s);

  /**
   *
   *
   * \return the AS SAP provider exported by this RRC
   */
  LteAsSapProvider* GetAsSapProvider ();

  /**
   * set the Component Carrier Management SAP this RRC should interact with
   *
   * \param s the Component Carrier Management SAP Provider to be used by this RRC
   */
  void SetLteCcmRrcSapProvider (LteUeCcmRrcSapProvider * s);

  /**
   * Get the Component Carrier Management SAP offered by this RRC
   * \return s the Component Carrier Management SAP User interface offered to the
   *           carrier component selection algorithm by this RRC
   */
  LteUeCcmRrcSapUser* GetLteCcmRrcSapUser ();

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
   *
   * \return the C-RNTI of the user
   */
  uint16_t GetRnti () const;

  /**
   *
   * \return the CellId of the attached Enb
   */
  uint16_t GetCellId () const;

  /**
   * \return the uplink bandwidth in RBs
   */
  uint8_t GetUlBandwidth () const;

  /**
   * \return the downlink bandwidth in RBs
   */
  uint8_t GetDlBandwidth () const;

  /**
   * \return the downlink carrier frequency (EARFCN)
   */
  uint32_t GetDlEarfcn () const;

  /**
   * \return the uplink carrier frequency (EARFCN)
   */
  uint32_t GetUlEarfcn () const;

  /**
   *
   * \return the current state
   */
  State GetState () const;

  /**
   *
   *
   * \param val true if RLC SM is to be used, false if RLC UM/AM are to be used
   */
  void SetUseRlcSm (bool val);

  /**
   * TracedCallback signature for imsi, cellId and rnti events.
   *
   * \param [in] imsi
   * \param [in] cellId
   */
  typedef void (*CellSelectionTracedCallback)
    (uint64_t imsi, uint16_t cellId);

  /**
   * TracedCallback signature for imsi, cellId and rnti events.
   *
   * \param [in] imsi
   * \param [in] cellId
   * \param [in] rnti
   */
  typedef void (*ImsiCidRntiTracedCallback)
    (uint64_t imsi, uint16_t cellId, uint16_t rnti);

  /**
   * TracedCallback signature for MIBRecieved, Sib1Received and
   * HandoverStart events.
   *
   * \param [in] imsi
   * \param [in] cellId
   * \param [in] rnti
   * \param [in] otherCid
   */
  typedef void (*MibSibHandoverTracedCallback)
    (uint64_t imsi, uint16_t cellId, uint16_t rnti, uint16_t otherCid);

  /**
   * TracedCallback signature for state transition events.
   *
   * \param [in] imsi
   * \param [in] cellId
   * \param [in] rnti
   * \param [in] oldState
   * \param [in] newState
   */
  typedef void (*StateTracedCallback)
    (uint64_t imsi, uint16_t cellId, uint16_t rnti,
    State oldState, State newState);

  /**
    * TracedCallback signature for secondary carrier configuration events.
    *
    * \param [in] Pointer to UE RRC
    * \param [in] List of LteRrcSap::SCellToAddMod
    */
  typedef void (*SCarrierConfiguredTracedCallback)
    (Ptr<LteUeRrc>, std::list<LteRrcSap::SCellToAddMod>);

  /**
   * TracedCallback signature for uplink time alignment timeout events.
   *
   *
   * \param [in] imsi
   * \param [in] cellId
   * \param [in] rnti
   * \param [in] result
   */
  typedef void (*UplinkOutofSyncTracedCallback)
    (uint64_t imsi, uint16_t cellId, uint16_t rnti, std::string result);

  /**
   * TracedCallback signature for radio link failure events.
   *
   *
   * \param [in] imsi
   * \param [in] rnti
   * \param [in] cellId
   */
  typedef void (*RadioLinkFailureTracedCallback)
    (uint64_t imsi, uint16_t rnti, uint16_t cellId);

  /**
   * TracedCallback signature for in-sync and out-of-sync detection events.
   *
   *
   * \param [in] imsi
   * \param [in] rnti
   * \param [in] cellId
   * \param [in] type
   * \param [in] count
   */
  typedef void (*PhySyncDetectionTracedCallback)
    (uint64_t imsi, uint16_t rnti, uint16_t cellId, std::string type, uint16_t count);

  /**
   * TracedCallback signature for UE mobility change during cell reselection events.
   *
   *
   * \param [in] imsi
   * \param [in] reselectionTimerDuration
   * \param [in] qHyst
   * \param [in] type
   */
  typedef void (*UeMobilityStateChangedTracedCallback)
    (uint64_t imsi, Time reselectionTimerDuration, double qHyst, std::string type);


  /**
   * Force the UE to stop trying to connect when connection is not completely carried out
   *
   */
  void StopConnectionAttempt ();

  /**
   * Structure containing parameters
   * related to reselection. Used as
   * a parameter in 'CellReselection' Trace.
   *
   */
  struct ReselectionInfo
  {
    uint16_t currentCellId;
    uint16_t reselectedCellId;
    Time reselectionTimerDuration;
    uint16_t numOfReselections;
    double hyst;
  };

  /**
    * TracedCallback signature for cell reselection events.
    *
    *
    * \param [in] imsi
    * \param [in] rnti
    * \param [in] reselectionInfo
    */
  typedef void (*CellReselectionTracedCallback)
    (uint64_t imsi, uint16_t rnti, LteUeRrc::ReselectionInfo reselectionInfo);



private:
  // PDCP SAP methods
  /**
   * Receive PDCP SDU function
   *
   * \param params LtePdcpSapUser::ReceivePdcpSduParameters
   */
  void DoReceivePdcpSdu (LtePdcpSapUser::ReceivePdcpSduParameters params);

  // CMAC SAP methods
  /**
   * Set temporary cell rnti function
   *
   * \param rnti RNTI
   */
  void DoSetTemporaryCellRnti (uint16_t rnti);
  /// Notify random access successful function
  void DoNotifyRandomAccessSuccessful ();
  /// Notify random access failed function
  void DoNotifyRandomAccessFailed ();
  void DoNotifyRarReceived ();
  void DoNotifyContentionResolutionTimeout ();

  // LTE AS SAP methods
  /**
   * Set CSG white list function
   *
   * \param csgId CSG ID
   */
  void DoSetCsgWhiteList (uint32_t csgId);
  /**
   * Force camped on ENB function
   *
   * \param cellId the cell ID
   * \param dlEarfcn the DL EARFCN
   */
  void DoForceCampedOnEnb (uint16_t cellId, uint32_t dlEarfcn);
  /**
   * Start cell selection function
   *
   * \param dlEarfcn the DL EARFCN
   */
  void DoStartCellSelection (uint32_t dlEarfcn);
  /// Connect function
  void DoConnect ();
  /**
   * Send data function
   *
   * \param packet the packet
   * \param bid the BID
   */
  void DoSendData (Ptr<Packet> packet, uint8_t bid);
  /// Disconnect function
  void DoDisconnect ();

  // CPHY SAP methods
  /**
   * Receive master information block function
   *
   * \param cellId the cell ID
   * \param msg LteRrcSap::MasterInformationBlock
   */
  void DoRecvMasterInformationBlock (uint16_t cellId,
                                     LteRrcSap::MasterInformationBlock msg);
  /**
   * Receive system information block type 1 function
   *
   * \param cellId the cell ID
   * \param msg LteRrcSap::SystemInformationBlockType1
   */
  void DoRecvSystemInformationBlockType1 (uint16_t cellId,
                                          LteRrcSap::SystemInformationBlockType1 msg);
  /**
   * Report UE measurements function
   *
   * \param params LteUeCphySapUser::UeMeasurementsParameters
   */
  void DoReportUeMeasurements (LteUeCphySapUser::UeMeasurementsParameters params);

  // RRC SAP methods

  /**
   * Part of the RRC protocol. Implement the LteUeRrcSapProvider::CompleteSetup interface.
   * \param params the LteUeRrcSapProvider::CompleteSetupParameters
   */
  void DoCompleteSetup (LteUeRrcSapProvider::CompleteSetupParameters params);
  /**
   * Part of the RRC protocol. Implement the LteUeRrcSapProvider::RecvSystemInformation interface.
   * \param msg the LteRrcSap::SystemInformation
   */
  void DoRecvSystemInformation (LteRrcSap::SystemInformation msg);
  /**
   * Part of the RRC protocol. Implement the LteUeRrcSapProvider::RecvRrcConnectionSetup interface.
   * \param msg the LteRrcSap::RrcConnectionSetup
   */
  void DoRecvRrcConnectionSetup (LteRrcSap::RrcConnectionSetup msg);
  /**
   * Part of the RRC protocol. Implement the LteUeRrcSapProvider::RecvRrcConnectionReconfiguration interface.
   * \param msg the LteRrcSap::RrcConnectionReconfiguration
   */
  void DoRecvRrcConnectionReconfiguration (LteRrcSap::RrcConnectionReconfiguration msg);
  /**
   * Part of the RRC protocol. Implement the LteUeRrcSapProvider::RecvRrcConnectionReestablishment interface.
   * \param msg LteRrcSap::RrcConnectionReestablishment
   */
  void DoRecvRrcConnectionReestablishment (LteRrcSap::RrcConnectionReestablishment msg);
  /**
   * Part of the RRC protocol. Implement the LteUeRrcSapProvider::RecvRrcConnectionReestablishmentReject interface.
   * \param msg LteRrcSap::RrcConnectionReestablishmentReject
   */
  void DoRecvRrcConnectionReestablishmentReject (LteRrcSap::RrcConnectionReestablishmentReject msg);
  /**
   * Part of the RRC protocol. Implement the LteUeRrcSapProvider::RecvRrcConnectionRelease interface.
   * \param msg LteRrcSap::RrcConnectionRelease
   */
  void DoRecvRrcConnectionRelease (LteRrcSap::RrcConnectionRelease msg);
  /**
   * Part of the RRC protocol. Implement the LteUeRrcSapProvider::RecvRrcConnectionReject interface.
   * \param msg the LteRrcSap::RrcConnectionReject
   */
  void DoRecvRrcConnectionReject (LteRrcSap::RrcConnectionReject msg);

  /**
   * RRC CCM SAP USER Method
   * \param res
   */
  void DoComponentCarrierEnabling (std::vector<uint8_t> res);


  // INTERNAL METHODS

  /**
   * \brief Go through the list of measurement results, choose the one with the
   *        strongest RSRP, and tell PHY to synchronize to it.
   *
   * \warning This function is a part of the *initial cell selection* procedure,
   *          hence must be only executed during IDLE mode.
   */
  void SynchronizeToStrongestCell ();

  /**
   * \brief Performs cell selection evaluation to the current serving cell.
   *
   * \warning This function is a part of the *initial cell selection* procedure,
   *          hence must be only executed during IDLE mode and specifically
   *          during the state when the UE just received the first SIB1 message
   *          from the serving cell.
   *
   * This function assumes that the required information for the evaluation
   * procedure have been readily gathered, such as *measurement results*, MIB,
   * and SIB1. Please refer to the LTE module's Design Documentation for more
   * details on the evaluation process.
   *
   * If the cell passes the evaluation, the UE will immediately camp to it.
   * Otherwise, the UE will pick another cell and restart the cell selection
   * procedure.
   */
  void EvaluateCellForSelection ();

  /**
   * \brief Update the current measurement configuration #m_varMeasConfig.
   * \param mc measurements to be performed by the UE
   *
   * Implements Section 5.5.2 "Measurement configuration" of 3GPP TS 36.331.
   * The supported subfunctions are:
   * - Measurement object removal
   * - Measurement object addition/ modification
   * - Reporting configuration removal
   * - Reporting configuration addition/ modification
   * - Quantity configuration
   * - Measurement identity removal
   * - Measurement identity addition/ modification
   *
   * The subfunctions that will be invoked are determined by the content of
   * the given measurement configuration.
   *
   * Note the existence of some chain reaction behaviours:
   * - Removal of measurement object or reporting configuration also removes any
   *   impacted measurement identities.
   * - Removal of measurement identity also removes any associated *reporting
   *   entry* from #m_varMeasReportList.
   * - Modification to measurement object or reporting configuration also
   *   removes any reporting entries of the impacted measurement identities
   *   from #m_varMeasReportList.
   * - Modification to quantity configuration also removes all existing
   *   reporting entries from #m_varMeasReportList, regardless of measurement
   *   identity.
   *
   * Some unsupported features:
   * - List of neighbouring cells
   * - List of black cells
   * - CGI reporting
   * - Periodical reporting configuration
   * - Measurement gaps
   * - s-Measure
   * - Speed-dependent scaling
   *
   * \warning There is a possibility that the input argument (of type
   *          LteRrcSap::MeasConfig) may contain information in fields related
   *          to the unsupported features. In such case, the function will raise
   *          an error.
   *
   * The measurement configuration given as an argument is typically provided by
   * the serving eNodeB. It is transmitted through the RRC protocol when the UE
   * joins the cell, e.g., by connection establishment or by incoming handover.
   * The information inside the argument can be configured from the eNodeB side,
   * which would then equally apply to all other UEs attached to the same
   * eNodeB. See the LTE module's User Documentation for more information on
   * configuring this.
   *
   * \sa LteRrcSap::MeasConfig, LteUeRrc::m_varMeasReportList
   */
  void ApplyMeasConfig (LteRrcSap::MeasConfig mc);

  /**
   * \brief Keep the given measurement result as the latest measurement figures,
   *        to be utilised by UE RRC functions.
   * \param cellId the cell ID of the measured cell
   * \param rsrp measured RSRP value to be saved (in dBm)
   * \param rsrq measured RSRQ value to be saved (in dB)
   * \param useLayer3Filtering
   * \todo Remove the useLayer3Filtering argument
   *
   * Implements Section 5.5.3.2 "Layer 3 filtering" of 3GPP TS 36.331. *Layer-3
   * filtering* is applied to the given measurement results before saved to
   * #m_storedMeasValues. The filtering is however disabled when the UE is in
   * IDLE mode, i.e., saving unfiltered values.
   *
   * Layer-3 filtering is influenced by a filter coefficient, which determines
   * the strength of the filtering. This coefficient is provided by the active
   * *quantity configuration* in #m_varMeasConfig, which is configured by the
   * LteUeRrc::ApplyMeasConfig. Details on how the coefficient works and how to
   * modify it can be found in LTE module's Design Documentation.
   *
   * \sa LteUeRrc::m_storedMeasValues
   */
  void SaveUeMeasurements (uint16_t cellId, double rsrp, double rsrq,
                           bool useLayer3Filtering);

  /**
   * \brief keep the given measurement result as the latest measurement figures,
   *        to be utilised by UE RRC functions.
   * \param cellId the cell ID of the measured cell
   * \param rsrp measured RSRP value to be saved (in dBm)
   * \param rsrq measured RSRQ value to be saved (in dB)
   * \param useLayer3Filtering
   * \param componentCarrierId
   * \todo Remove the useLayer3Filtering argument
   *
   * As for SaveUeMeasurements, this function aims to store the latest measurements
   * related to the secondary component carriers.
   * in the current implementation it saves only measurements related on the serving
   * secondary carriers while, measurements related to the Neighbor Cell are filtered
   */

  void SaveScellUeMeasurements (uint16_t cellId, double rsrp, double rsrq,
                                bool useLayer3Filtering, uint16_t componentCarrierId);
  /**
   * \brief Evaluate the reporting criteria of a measurement identity and
   *        invoke some reporting actions based on the result.
   * \param measId the measurement identity to be evaluated
   *
   * Implements Section 5.5.4.1 "Measurement report triggering - General" of
   * 3GPP TS 36.331. This function take into use the latest measurement results
   * and evaluate them against the *entering condition* and the *leaving
   * condition* of the measurement identity's reporting criteria. The evaluation
   * also take into account other defined criteria, such as *hysteresis* and
   * *time-to-trigger*.
   *
   * The entering and leaving condition to be evaluated are determined by the
   * *event type* of the measurement identity's reporting criteria. As defined
   * in LteRrcSap::ReportConfigEutra, there 5 supported events. The gore details
   * of these events can be found in Section 5.5.4 of 3GPP TS 36.331.
   *
   * An applicable entering condition (i.e., the condition evaluates to true)
   * will insert a new *reporting entry* to #m_varMeasReportList, so
   * *measurement reports* would be produced and submitted to eNodeB. On the
   * other hand, an applicable leaving condition will remove the related
   * reporting entry from #m_varMeasReportList, so submission of related
   * measurement reports to eNodeB will be suspended.
   */
  void MeasurementReportTriggering (uint8_t measId);

  /**
   * \brief Produce a proper measurement report from the given measurement
   *        identity's reporting entry in #m_varMeasReportList and then submit
   *        it to the serving eNodeB.
   * \param measId the measurement identity which report is to be submitted.
   *
   * Implements Section 5.5.5 "Measurement reporting" of 3GPP TS 36.331.
   * Producing a *measurement report* involves several tasks such as:
   * - including the measurement results of the serving cell into the report;
   * - selecting some neighbour cells which triggered the reporting (i.e., those
   *   in *cellsTriggeredList*) to be included in the report;
   * - sorting the order of neighbour cells in the report by their RSRP or RSRQ
   *   measurement results (the highest comes first); and
   * - ensuring the number of neighbour cells in the report is under the
   *   *maxReportCells* limit defined by the measurement identity's reporting
   *   configuration.
   *
   * The RSRP and RSRQ measurement results included in the report are expressed
   * in 3GPP-specified range format. They are converted from dBm and dB units
   * using EutranMeasurementMapping::Dbm2RsrpRange and
   * EutranMeasurementMapping::Db2RsrqRange functions.
   *
   * Measurement report is submitted to the serving eNodeB through the *RRC
   * protocol*. The LteUeRrcSapUser::SendMeasurementReport method of the *UE RRC
   * SAP* facilitates this submission.
   *
   * After the submission, the function will repeat itself after a certain
   * interval. The interval length may vary from 120 ms to 60 minutes and is
   * determined by the *report interval* parameter specified by the measurement
   * identity's reporting configuration.
   */
  void SendMeasurementReport (uint8_t measId);

  /**
   * Apply radio resource config dedicated.
   * \param rrcd LteRrcSap::RadioResourceConfigDedicated
   */
  void ApplyRadioResourceConfigDedicated (LteRrcSap::RadioResourceConfigDedicated rrcd);
  /**
   * Apply radio resource config dedicated secondary carrier.
   * \param nonCec LteRrcSap::NonCriticalExtensionConfiguration
   */
  void ApplyRadioResourceConfigDedicatedSecondaryCarrier (LteRrcSap::NonCriticalExtensionConfiguration nonCec);
  /// Start connetion function
  void StartConnection ();
  /// Leave connected mode
  void LeaveConnectedMode ();
  /// Dispose old SRB1
  void DisposeOldSrb1 ();
  /**
   * Bid 2 DR bid.
   * \param bid the BID
   * \returns the DR bid
   */
  uint8_t Bid2Drbid (uint8_t bid);
  /**
   * Switch the UE RRC to the given state.
   * \param s the destination state
   */
  void SwitchToState (State s);

  std::map<uint8_t, uint8_t> m_bid2DrbidMap; ///< bid to DR bid map

  std::vector<LteUeCphySapUser*> m_cphySapUser; ///< UE CPhy SAP user
  std::vector<LteUeCphySapProvider*> m_cphySapProvider; ///< UE CPhy SAP provider

  std::vector<LteUeCmacSapUser*> m_cmacSapUser; ///< UE CMac SAP user
  std::vector<LteUeCmacSapProvider*> m_cmacSapProvider; ///< UE CMac SAP provider

  LteUeRrcSapUser* m_rrcSapUser; ///< RRC SAP user
  LteUeRrcSapProvider* m_rrcSapProvider; ///< RRC SAP provider

  LteMacSapProvider* m_macSapProvider; ///< MAC SAP provider
  LtePdcpSapUser* m_drbPdcpSapUser; ///< DRB PDCP SAP user

  LteAsSapProvider* m_asSapProvider; ///< AS SAP provider
  LteAsSapUser* m_asSapUser; ///< AS SAP user

  // Receive API calls from the LteUeComponentCarrierManager  instance.
  // LteCcmRrcSapUser* m_ccmRrcSapUser;
  /// Interface to the LteUeComponentCarrierManage instance.
  LteUeCcmRrcSapProvider* m_ccmRrcSapProvider; ///< CCM RRC SAP provider
  LteUeCcmRrcSapUser* m_ccmRrcSapUser; ///< CCM RRC SAP user

  /// The current UE RRC state.
  State m_state;

  /// The unique UE identifier.
  uint64_t m_imsi;
  /**
   * The `C-RNTI` attribute. Cell Radio Network Temporary Identifier.
   */
  uint16_t m_rnti;
  /**
   * The `CellId` attribute. Serving cell identifier.
   */
  uint16_t m_cellId;

  /**
   * The `Srb0` attribute. SignalingRadioBearerInfo for SRB0.
   */
  Ptr<LteSignalingRadioBearerInfo> m_srb0;
  /**
   * The `Srb1` attribute. SignalingRadioBearerInfo for SRB1.
   */
  Ptr<LteSignalingRadioBearerInfo> m_srb1;
  /**
   * SRB1 configuration before RRC connection reconfiguration. To be deleted
   * soon by DisposeOldSrb1().
   */
  Ptr<LteSignalingRadioBearerInfo> m_srb1Old;
  /**
   * The `DataRadioBearerMap` attribute. List of UE RadioBearerInfo for Data
   * Radio Bearers by LCID.
   */
  std::map <uint8_t, Ptr<LteDataRadioBearerInfo> > m_drbMap;

  /**
   * True if RLC SM is to be used, false if RLC UM/AM are to be used.
   * Can be modified using SetUseRlcSm().
   */
  bool m_useRlcSm;

  uint8_t m_lastRrcTransactionIdentifier; ///< last RRC transaction identifier

  LteRrcSap::PdschConfigDedicated m_pdschConfigDedicated; ///< the PDSCH condig dedicated

  uint8_t m_dlBandwidth; /**< Downlink bandwidth in RBs. */
  uint8_t m_ulBandwidth; /**< Uplink bandwidth in RBs. */

  uint32_t m_dlEarfcn;  /**< Downlink carrier frequency. */
  uint32_t m_ulEarfcn;  /**< Uplink carrier frequency. */
  std::list<LteRrcSap::SCellToAddMod> m_sCellToAddModList; /**< Secondary carriers. */

  /**
   * The `MibReceived` trace source. Fired upon reception of Master Information
   * Block. Exporting IMSI, the serving cell ID, RNTI, and the source cell ID.
   */
  TracedCallback<uint64_t, uint16_t, uint16_t, uint16_t> m_mibReceivedTrace;
  /**
   * The `Sib1Received` trace source. Fired upon reception of System
   * Information Block Type 1. Exporting IMSI, the serving cell ID, RNTI, and
   * the source cell ID.
   */
  TracedCallback<uint64_t, uint16_t, uint16_t, uint16_t> m_sib1ReceivedTrace;
  /**
   * The `Sib2Received` trace source. Fired upon reception of System
   * Information Block Type 2. Exporting IMSI, the serving cell ID, RNTI.
   */
  TracedCallback<uint64_t, uint16_t, uint16_t> m_sib2ReceivedTrace;
  /**
   * The `StateTransition` trace source. Fired upon every UE RRC state
   * transition. Exporting IMSI, the serving cell ID, RNTI, old state, and new
   * state.
   */
  TracedCallback<uint64_t, uint16_t, uint16_t, State, State> m_stateTransitionTrace;
  /**
   * The `InitialCellSelectionEndOk` trace source. Fired upon successful
   * initial cell selection procedure. Exporting IMSI and the selected cell ID.
   */
  TracedCallback<uint64_t, uint16_t> m_initialCellSelectionEndOkTrace;
  /**
   * The `InitialCellSelectionEndError` trace source. Fired upon failed initial
   * cell selection procedure. Exporting IMSI and the cell ID under evaluation.
   */
  TracedCallback<uint64_t, uint16_t> m_initialCellSelectionEndErrorTrace;
  /**
   * The `RandomAccessSuccessful` trace source. Fired upon successful
   * completion of the random access procedure. Exporting IMSI, cell ID, and
   * RNTI.
   */
  TracedCallback<uint64_t, uint16_t, uint16_t> m_randomAccessSuccessfulTrace;
  /**
   * The `RandomAccessError` trace source. Fired upon failure of the random
   * access procedure. Exporting IMSI, cell ID, and RNTI.
   */
  TracedCallback<uint64_t, uint16_t, uint16_t> m_randomAccessErrorTrace;
  /**
   * The `ConnectionEstablished` trace source. Fired upon successful RRC
   * connection establishment. Exporting IMSI, cell ID, and RNTI.
   */
  TracedCallback<uint64_t, uint16_t, uint16_t> m_connectionEstablishedTrace;
  /**
   * The `ConnectionTimeout` trace source. Fired upon timeout RRC connection
   * establishment because of T300. Exporting IMSI, cell ID, and RNTI.
   */
  TracedCallback<uint64_t, uint16_t, uint16_t> m_connectionTimeoutTrace;
  /**
   * The `ConnectionReconfiguration` trace source. Fired upon RRC connection
   * reconfiguration. Exporting IMSI, cell ID, and RNTI.
   */
  TracedCallback<uint64_t, uint16_t, uint16_t> m_connectionReconfigurationTrace;
  /**
   * The `HandoverStart` trace source. Fired upon start of a handover
   * procedure. Exporting IMSI, source cell ID, RNTI, and target cell ID.
   */
  TracedCallback<uint64_t, uint16_t, uint16_t, uint16_t> m_handoverStartTrace;
  /**
   * The `HandoverEndOk` trace source. Fired upon successful termination of a
   * handover procedure. Exporting IMSI, cell ID, and RNTI.
   */
  TracedCallback<uint64_t, uint16_t, uint16_t> m_handoverEndOkTrace;
  /**
   * The `HandoverEndError` trace source. Fired upon failure of a handover
   * procedure. Exporting IMSI, cell ID, and RNTI.
   */
  TracedCallback<uint64_t, uint16_t, uint16_t> m_handoverEndErrorTrace;
  /**
   * The `SCarrierConfigured` trace source. Fired after the configuration
   * of secondary carriers received through RRC Connection Reconfiguration
   * message.
   */
  TracedCallback<Ptr<LteUeRrc>, std::list<LteRrcSap::SCellToAddMod> > m_sCarrierConfiguredTrace;

  /**
   * The 'UplinkOutofSync' trace source. Fired upon time alignment timer timeout.
   *
   */
  TracedCallback<uint64_t, uint16_t, uint16_t, std::string> m_uplinkOutofSyncTrace;

  /**
   * The 'PhySyncDetection' trace source. Fired when UE RRC
   * receives in-sync or out-of-sync indications from UE PHY
   *
   */
  TracedCallback<uint64_t, uint16_t, uint16_t, std::string, uint16_t> m_phySyncDetectionTrace;

  /**
   * The 'RadioLinkFailure' trace source. Fired when T310 timer expires.
   *
   */
  TracedCallback<uint64_t, uint16_t, uint16_t> m_radioLinkFailureTrace;

  /// True if a connection request by upper layers is pending.
  bool m_connectionPending;
  /// True if MIB was received for the current cell.
  bool m_hasReceivedMib;
  /// True if SIB1 was received for the current cell.
  bool m_hasReceivedSib1;
  /// True if SIB2 was received for the current cell.
  bool m_hasReceivedSib2;

  /// Stored content of the last SIB1 received.
  LteRrcSap::SystemInformationBlockType1 m_lastSib1;

  /// List of cell ID of acceptable cells for cell selection that have been detected.
  std::set<uint16_t> m_acceptableCell;

  /// List of CSG ID which this UE entity has access to.
  uint32_t m_csgWhiteList;


  // INTERNAL DATA STRUCTURE RELATED TO UE MEASUREMENTS

  /**
   * \brief Includes the accumulated configuration of the measurements to be
   *        performed by the UE.
   *
   * Based on 3GPP TS 36.331 section 7.1. Also note that some optional variables
   * in the specification are omitted.
   */
  struct VarMeasConfig
  {
    std::map<uint8_t, LteRrcSap::MeasIdToAddMod> measIdList; ///< measure ID list
    std::map<uint8_t, LteRrcSap::MeasObjectToAddMod> measObjectList; ///< measure object list
    std::map<uint8_t, LteRrcSap::ReportConfigToAddMod> reportConfigList; ///< report config list
    LteRrcSap::QuantityConfig quantityConfig; ///< quantity config
    double aRsrp; ///< RSRP
    double aRsrq; ///< RSRQ
  };

  /**
   * \brief Includes the accumulated configuration of the measurements to be
   *        performed by the UE.
   *
   * Based on 3GPP TS 36.331 section 7.1.
   */
  VarMeasConfig m_varMeasConfig;

  /**
   * \brief Represents a single measurement reporting entry., which includes
   *        information about a measurement for which the triggering conditions
   *        have been met.
   *
   * Based on 3GPP TS 36.331 section 7.1.
   */
  struct VarMeasReport
  {
    uint8_t measId; ///< measure ID
    std::set<uint16_t> cellsTriggeredList; ///< note: only E-UTRA is supported.
    uint32_t numberOfReportsSent; ///< number of reports sent
    EventId periodicReportTimer; ///< periodic report timer
  };

  /**
   * \brief The list of active reporting entries, indexed by the measurement
   *        identity which triggered the reporting. Includes information about
   *        measurements for which the triggering conditions have been met.
   */
  std::map<uint8_t, VarMeasReport> m_varMeasReportList;

  /**
   * \brief List of cell IDs which are responsible for a certain trigger.
   */
  typedef std::list<uint16_t> ConcernedCells_t;

  /**
   * \brief Compose a new reporting entry of the given measurement identity,
   *        insert it into #m_varMeasReportList, and set it up for submission
   *        to eNodeB.
   * \param measId the measurement identity which the new reporting entry will
   *               be based upon
   * \param enteringCells the cells which are responsible for triggering the
   *                      reporting (i.e., successfully fulfilling the entering
   *                      condition of the measurement identity) and will be
   *                      included in the measurement report.
   *
   * \note If an existing reporting entry with the same measurement identity has
   *       already existed in #m_varMeasReportList, the function will update it
   *       by adding the entering cells into the existing reporting entry.
   * \note When time-to-trigger is enabled for this measurement identity, the
   *       function will also remove the related trigger from the
   *       #m_enteringTriggerQueue.
   */
  void VarMeasReportListAdd (uint8_t measId, ConcernedCells_t enteringCells);

  /**
   * \brief Remove some cells from an existing reporting entry in
   *        #m_varMeasReportList.
   * \param measId the measurement identity to be removed from
   *               #m_varMeasReportList, must already exists there, otherwise
   *               an error would be raised
   * \param leavingCells the cells which are about to be removed
   * \param reportOnLeave when true, will make the function send one last
   *                      measurement report to eNodeB before removing it
   *
   * \note If a given cell is not found in the reporting entry, the function
   *       will quietly continue.
   * \note If the removal has removed all the cells in the reporting entry, the
   *       function will remove the reporting entry as well.
   * \note When time-to-trigger is enabled for this measurement identity, the
   *       function will also remove the related trigger from the
   *       #m_leavingTriggerQueue.
   */
  void VarMeasReportListErase (uint8_t measId, ConcernedCells_t leavingCells,
                               bool reportOnLeave);

  /**
   * \brief Remove the reporting entry of the given measurement identity from
   *        #m_varMeasReportList.
   * \param measId the measurement identity to be removed from
   *               #m_varMeasReportList, must already exists there, otherwise
   *               an error would be raised
   *
   * Any events or triggers related with this measurement identity will be
   * canceled as well.
   */
  void VarMeasReportListClear (uint8_t measId);

  /**
   * \brief Represents a measurement result from a certain cell.
   */
  struct MeasValues
  {
    double rsrp; ///< Measured RSRP in dBm.
    double rsrq; ///< Measured RSRQ in dB.
    Time timestamp; ///< Not used. \todo Should be removed.
  };

  /**
   * \brief Internal storage of the latest measurement results from all detected
   *        detected cells, indexed by the cell ID where the measurement was
   *        taken from.
   *
   * Each *measurement result* comprises of RSRP (in dBm) and RSRQ (in dB).
   *
   * In IDLE mode, the measurement results are used by the *initial cell
   * selection* procedure. While in CONNECTED mode, *layer-3 filtering* is
   * applied to the measurement results and they are used by *UE measurements*
   * function (LteUeRrc::MeasurementReportTriggering and
   * LteUeRrc::SendMeasurementReport).
   */
  std::map<uint16_t, MeasValues> m_storedMeasValues;

  /**
   * \brief Stored measure values per carrier.
   */
  std::map<uint16_t, std::map <uint8_t, MeasValues> > m_storedMeasValuesPerCarrier;

  /**
   * \brief Internal storage of the latest measurement results from all detected
   *        detected Secondary carrier component, indexed by the carrier component ID
   *        where the measurement was taken from.
   *
   * Each *measurement result* comprises of RSRP (in dBm) and RSRQ (in dB).
   *
   * In IDLE mode, the measurement results are used by the *initial cell
   * selection* procedure. While in CONNECTED mode, *layer-3 filtering* is
   * applied to the measurement results and they are used by *UE measurements*
   * function:
   * - LteUeRrc::MeasurementReportTriggering: in this case it is not set any
   *   measurement related to seconday carrier components since the
   *   A6 event is not implemented
   * - LteUeRrc::SendMeasurementReport: in this case the report are sent.
   */
  std::map<uint16_t, MeasValues> m_storedScellMeasValues;

  /**
   * \brief Represents a single triggered event from a measurement identity
   *        which reporting criteria have been fulfilled, but delayed by
   *        time-to-trigger.
   */
  struct PendingTrigger_t
  {
    uint8_t measId; ///< The measurement identity which raised the trigger.
    ConcernedCells_t concernedCells; ///< The list of cells responsible for this trigger.
    EventId timer; ///< The pending reporting event, scheduled at the end of the time-to-trigger.
  };

  /**
   * \brief List of triggers that were raised because entering condition have
   *        been true, but are still delayed from reporting it by
   *        time-to-trigger.
   *
   * The list is indexed by the measurement identity where the trigger
   * originates from. The enclosed event will run at the end of the
   * time-to-trigger and insert a *reporting entry* to #m_varMeasReportList.
   */
  std::map<uint8_t, std::list<PendingTrigger_t> > m_enteringTriggerQueue;

  /**
   * \brief List of triggers that were raised because leaving condition have
   *        been true, but are still delayed from stopping the reporting by
   *        time-to-trigger.
   *
   * The list is indexed by the measurement identity where the trigger
   * originates from. The enclosed event will run at the end of the
   * time-to-trigger and remove the associated *reporting entry* from
   * #m_varMeasReportList.
   */
  std::map<uint8_t, std::list<PendingTrigger_t> > m_leavingTriggerQueue;

  /**
   * \brief Clear all the waiting triggers in #m_enteringTriggerQueue which are
   *        associated with the given measurement identity.
   * \param measId the measurement identity to be processed, must already exists
   *               in #m_enteringTriggerQueue, otherwise an error would be
   *               raised
   *
   * \note The function may conclude that there is nothing to be removed. In
   *       this case, the function will simply ignore quietly.
   *
   * This function is used when the entering condition of the measurement
   * identity becomes no longer true. Therefore all the waiting triggers for
   * this measurement identity in #m_enteringTriggerQueue have become invalid
   * and must be canceled.
   *
   * \sa LteUeRrc::m_enteringTriggerQueue
   */
  void CancelEnteringTrigger (uint8_t measId);

  /**
   * \brief Remove a specific cell from the waiting triggers in
   *        #m_enteringTriggerQueue which belong to the given measurement
   *        identity.
   * \param measId the measurement identity to be processed, must already exists
   *               in #m_enteringTriggerQueue, otherwise an error would be
   *               raised
   * \param cellId the cell ID to be removed from the waiting triggers
   *
   * \note The function may conclude that there is nothing to be removed. In
   *       this case, the function will simply ignore quietly.
   *
   * This function is used when a specific neighbour cell no longer fulfills
   * the entering condition of the measurement identity. Thus the cell must be
   * removed from all the waiting triggers for this measurement identity in
   * #m_enteringTriggerQueue.
   *
   * \sa LteUeRrc::m_enteringTriggerQueue
   */
  void CancelEnteringTrigger (uint8_t measId, uint16_t cellId);

  /**
   * \brief Clear all the waiting triggers in #m_leavingTriggerQueue which are
   *        associated with the given measurement identity.
   * \param measId the measurement identity to be processed, must already exists
   *               in #m_leavingTriggerQueue, otherwise an error would be
   *               raised
   *
   * \note The function may conclude that there is nothing to be removed. In
   *       this case, the function will simply ignore quietly.
   *
   * This function is used when the leaving condition of the measurement
   * identity becomes no longer true. Therefore all the waiting triggers for
   * this measurement identity in #m_leavingTriggerQueue have become invalid
   * and must be canceled.
   *
   * \sa LteUeRrc::m_leavingTriggerQueue
   */
  void CancelLeavingTrigger (uint8_t measId);

  /**
   * \brief Remove a specific cell from the waiting triggers in
   *        #m_leavingTriggerQueue which belong to the given measurement
   *        identity.
   * \param measId the measurement identity to be processed, must already exists
   *               in #m_leavingTriggerQueue, otherwise an error would be
   *               raised
   * \param cellId the cell ID to be removed from the waiting triggers
   *
   * \note The function may conclude that there is nothing to be removed. In
   *       this case, the function will simply ignore quietly.
   *
   * This function is used when a specific neighbour cell no longer fulfills
   * the leaving condition of the measurement identity. Thus the cell must be
   * removed from all the waiting triggers for this measurement identity in
   * #m_leavingTriggerQueue.
   *
   * \sa LteUeRrc::m_leavingTriggerQueue
   */
  void CancelLeavingTrigger (uint8_t measId, uint16_t cellId);

  /**
   * The `T300` attribute. Timer for RRC connection establishment procedure
   * (i.e., the procedure is deemed as failed if it takes longer than this).
   * See Section 7.3 of 3GPP TS 36.331.
   */
  Time m_t300;

  /**
   * \brief Invokes ConnectionEstablishmentTimeout() if RRC connection
   *        establishment procedure for this UE takes longer than T300.
   */
  EventId m_connectionTimeout;

  /**
   * \brief Invoked after timer T300 expires, notifying upper layers that RRC
   *        connection establishment procedure has failed.
   */
  void ConnectionTimeout ();

public:
  /**
   * The number of component carriers.
   */
  uint16_t m_numberOfComponentCarriers;

  /**
   * Resets the UE back to the camped state upon expiry of the inactivity 
   * timer at the eNodeB. Since this is not an abnormal failure 
   * case (such as RLF), the UE camps on the last cell for which it was in 
   * RRC_CONNECTED state. At RRC, measurement reports are cleared
   * and the appropriate flags are reset to their
   * default values. This method in turn triggers the reset methods of
   * UE PHY, MAC and NAS layers.
   */
  void DoResetToCamped ();

  /**
   * This method is called after UE has camped on the current cell for
   * atleast a minimum of 1s and every time the method 'DoReportUeMeasurements'
   * is triggered (i.e every 200ms). If the UE is in camped state and if the current
   * cell signal strength falls below the threshold, then cell re-selection procedure
   * is started. All the cells are ranked based on their signal strengths
   * with appropriate offsets applied and if the cell re-selection criterion (R criterion)
   * is satisfied (if a neighbor cell signal strength exceeds the serving cell signal strength),
   * the strongest neighbor is selected and cell re-selection timer is started. After the
   * timer expires, if the condition is satisfied then UE re-selects to the new cell.
   * (Specified in 3GPP TS 36.304 5.2.4)
   *
   */
  void DoStartCellReselection ();

  /**
   * This method is executed when the cell re-selection timer expires. If the
   * cell re-selection criterion is still satisfied and the UE remains in camped state,
   * then the cell re-selection is performed (UE re-selects to the new cell). The number
   * of re-selections is also counted, to apply speed dependent scaling rules when required.
   * The UE shall not count consecutive re-selections between same two cells into mobility
   * state detection criteria if same cell is reselected just after one other re-selection.
   *
   *
   * \param RnCellId cell ID of the neighbor cell with the strongest signal strength
   */
  void EvaluateCellforReselection (uint16_t rnCellId);

  /**
   * This method is executed when the time alignment timer
   * expires at the MAC layer. The UE is considered to be out of sync
   * only in the RRC CONNECTED_NORMALLY state and if the RRC connection reconfiguration
   * message has been received. If the UE is in any other
   * state, then the UE is considered to be still uplink time aligned
   * even though the timer has expired in order to reduce unwanted errors
   * (such as in case of handover). If UE is out of sync, then the HARQ buffers
   * are cleared, SRS configuration is released and also uplink grants and downlink
   * assignments are cleared. No message or data transmission takes place in this
   * state other than the RA preamble transmission.
   *
   *
   * \return true if the UE is considered to be uplink out of sync
   */
  bool DoNotifyTimeAlignmentTimeout ();

  /**
   * Notify the eNodeB to start the parallel time alignment timer of
   * the UE identified by its RNTI when RAR or TAC is received by the UE.
   * This notification is sent in an ideal way through the RRC protocol
   * to maintain synchronization between the 2 timers.
   *
   *
   * \param timeAlignmentTimer the duration of the timer
   * \param rnti the RNTI of the UE whose timer has to be restarted
   */
  void DoNotifyEnbTimeAlignmentTimerToStart (Time timeAlignmentTimer, uint16_t rnti);

  /**
   * When the UE receives the paging message, the UE identity in the
   * message is matched with the identity of the this UE (here IMSI is used).
   * If it matches, then the paging message is accepted. The RRC connection is established
   * only if the UE is in the camped state. If not, after the MME paging timer expires,
   * the eNodeB tries to page the UE again.
   *
   *
   * \param msg RRC paging message
   */
  void DoRecvPagingMsg (LteRrcSap::RrcPagingMessage msg);

  /**
   * Set the flag when uplink data arrives and the UE is in
   * camped state. Here, the uplink data is buffered. The buffered
   * data packets are sent towards the eNodeB after the
   * RRC connection is established.
   *
   *
   * \param val the flag is set to true if uplink data arrives in camped state
   */
  void DoSetUlDataPendingFlag (bool val);

  /**
   * If the UE is in the camped state, then a true value is returned.
   * This indicates to the NAS layer that the UE triggered service request
   * can be started upon uplink data arrival i.e the RRC connection establishment
   * is started from the UE side without the need for a paging message.
   *
   *
   * \return true if the RRC connection can be established
   */
  bool IsUeCamped ();

  /**
   * Set the type of RACH model being used.
   * Required to prevent the check for UE contention resolution msg
   * reception for ideal RACH.
   *
   * \param realPrach true, if realistic RACH model is used
   */
  void SetPrachMode (bool realPrach);

  /**
   * Set the m_connectionPending flag to true to enable
   * the UE to transition to CONNECTED state for LTE only simulations
   *
   * \param connectionPending true, if the flag is to be set
   */
  void SetConnectionPendingFlag (bool connectionPending);

  /**
   * Resets the UE back to the IDLE_CELL_SEARCH state upon
   * abnormal failure conditions such as RA failure, radio link failure 
   * or when RRC connection release message is received 
   * (as stated in 3GPP TS 36.331 5.3.12 and TS 36.304 5.2.7). 
   * At RRC, measurement reports are cleared and the appropriate 
   * flags are reset to their default values. This method in turn 
   * triggers the reset methods of UE PHY, MAC and NAS layers.
   * The initial cell selection procedure is triggered upon reaching 
   * this RRC state. Once in the IDLE_CELL_SEARCH state, the UE 
   * attempts the cell selection procedure to camp on a suitable cell.
   * Initially, the UE is reset to camped state and the cell selection procedure
   * starts at the next full millisecond to avoid spectrum model mismatch error 
   * which occurs when the DL data or ctrl info is sent from eNB before the cell 
   * selection procedure is started. 
   */
  void DoResetToIdle ();

  /**
   * Perfoms the cell selection procedure after the RRC connection 
   * between UE and eNB is released and the UE is reset to camped state 
   * (camps on the same cell in which it was during connected state) due 
   * to any link failure is detected. If reseting was due to an external 
   * factor (not due to any failure), then the UE is deactivated (
   * UeRrc:IDLE_START, UeNas:OFF state) and no cell selection process is
   * started (cell selection process starts when UE is reactivated again)
   */
  void DoCellSelectionDueToFailure();

  /**
   * Deactivates the UE when it is not needed to be used. This method
   * resets the UE to (UeRrc:IDLE_START, UeNas:OFF) state and the RRC
   * connection is released between UE and eNodeB. The UE will be in an 
   * inactive state and it cannot transmit/receive ctrl & data info until the 
   * UE is reactivated again.
   * **Imp: when this method is called, the UE context at the eNB should also 
   * be removed if it exits by calling LteEnbRrc::RemoveUeContextAtEnodeb
   * method, to avoid errors.
   * 
   * This method is normally called from outside the LTE module 
   * (ex: sim script) and can be used together with the ReactivateUe method
   * to control UE deactivation and reactivation
   * 
   * Ex: When running simulations with SUMO, the UE can be deactivated
   * from the simulation script when the UE/car leaves the road network
   */
  void ShutdownUe();

  /**
   * Reactivates the UE when it has to be used again. The initial
   * cell selection procedure is triggered so that the UE can camp
   * on a suitable cell.
   * 
   * This method is normally called from outside the LTE module 
   * (ex: sim script) and can be used together with the ShutdownUe method
   * to control UE reactivation and deactivation  
   * 
   * Ex: When running simulations with SUMO, the UE can be reactivated
   * from the simulation script when the UE/car enters the road network
   */
  void ReactivateUe();

private:
  /**
   * Set the cell reselection timer value
   *
   *
   * \param reselectionTimerValue the duration of the reselection timer
   */
  void SetReselectionTimerValue (Time reselectionTimerValue);

  /**
   * Set the default value of QHyst required for cell reselection.
   *
   *
   * \param qHyst the hysteresis value for ranking criteria
   */
  void SetQHyst (double qHyst);

  /**
   * Based on the number of cell re-selections performed
   * during the time period tCRMax, the mobility
   * state of the UE is changed and the appropriate scaling factors
   * are applied to t-ReselectionEUTRA and q-Hyst values.
   * This enables the cell re-selection procedure to be triggered and
   * performed faster for UEs traveling at high speeds and ensure
   * that the UE always camps on the strongest cell.
   * (Specified in 3GPP TS 36.304 5.2.4.3)
   *
   */
  void EvaluateUeMobility ();

  /**
   * After the time period tCRMax has elapsed, if criteria for neither
   * Medium- or High-mobility state is detected during time period TCRmaxHyst,
   * the UE enters normal mobility state.
   *
   */
  void SwitchToNormalMobility ();

  /**
   * Upon detection of radio link failure, the UE leaves the RRC_CONNECTED 
   * state (transitions from CONNECTED_NORMALLY to IDLE_CELL_SEARCH) and 
   * perfoms cell selection to camp on a suitable cell (as per 3GPP TS 36.331
   * 5.3.11.3). The eNodeB is notified in an ideal way to release 
   * the UE context since there is no radio link failure detection 
   * implemented at the eNodeB. If the deletion process is not synchronous, 
   * then errors occur due to triggering of assert messages.
   */
  void RadioLinkFailureDetected ();

  /**
   * Triggered upon receiving an in sync indication from UE PHY.
   * When the count equals N311, then T310 is cancelled.
   *
   */
  void DoNotifyInSync ();

  /**
   * Triggered upon receiving an out of sync indication from UE PHY.
   * When the count equals N310, then T310 is started.
   *
   */
  void DoNotifyOutOfSync ();

  /**
   * Reset the number of out-of-sync or in-sync indications 
   * received by the RRC layer to zero when the out-of-sync 
   * or in-sync condition is not fulfilled during its evaluation
   * respectively. During radio link failure detection, consecutive 
   * reception of 'N310' out-of-sync indications and also consecutive 
   * reception of 'N311' in-sync indications is required. 
   * Refer 3GPP TS 36.331 5.3.11.1 and 5.3.11.2.
   */
  void DoResetNumOfSyncIndications(); 

  /**
   * Receive notification from MAC that
   * CriLteControlMessage was received
   *
   */
  void DoNotifyContentionResolutionMsgReceived ();

  /**
   * This specifies the cell reselection timer value.
   * See 3GPP 36.304 section 5.2.4.7,specified by parameter TReselectionRat.
   */
  Time m_reselectionTimer;

  /**
   * After the time limit specified by m_reselectionTimer
   * is passed, this event is executed to evaluated
   * the strongest neighbouring cell for reselection
   */
  EventId m_reselectionTimeout;
  bool m_UlDataPendingFlag; ///< true if the buffer in NAS contains packets to be transmitted in uplink direction
  uint16_t m_sIntraSearch; ///< This specifies the Srxlev threshold (in dB) for intra-frequency measurements needed for cell reselection
  double m_qHyst; ///< This specifies the hysteresis value for ranking criteria.
  double m_qHystNormal; ///< This specifies the hysteresis value for ranking criteria during normal mobility
  double m_qHystSfMedium; ///< This specifies the hysteresis value for ranking criteria during medium mobility
  double m_qHystSfHigh; ///< This specifies the hysteresis value for ranking criteria during high mobility
  double m_qOffsetCell; ///<  This specifies the offset between the two cells
  Time m_tCrMax; ///< This specifies the duration for evaluating allowed amount of cell reselection(s)
  Time m_tCrMaxHyst; ///< This specifies the additional time period before the UE can enter Normal-mobility state.
  uint8_t m_nCrM; ///< This specifies the maximum number of cell reselections to enter Medium-mobility state
  uint8_t m_nCrH; ///< This specifies the maximum number of cell reselections to enter High-mobility state
  double m_tReselectionEutraSfMedium; ///< This specifies scaling factor for Qhyst for Medium-mobility state
  double m_tReselectionEutraSfHigh; ///< This specifies scaling factor for Qhyst for High-mobility state
  uint16_t m_numOfReselections; ///< Counter to count the num of reselections and evaluate mobility
  std::pair<uint16_t, uint16_t> m_campedCellIds; ///< pair of previous and present camped cells for a UE
  EventId m_reselecEvaluationTimeout; ///< UE mobility is evaluated when event expires to perform mobility dependent cell reselection
  Time m_reselectionTimerNormal; ///< Cell reselection timer value for normal mobility

  /**
   * The 'T310' attribute. After detecting N310 out-of-sync indications,
   * if number of in-sync indications detected is less than N311 before this
   * time, then the radio link is considered to have failed and the UE
   * transitions to IDLE_CELL_SEARCH state to camp on a suitable cell
   * and UE context at eNodeB is destroyed. RRC connection reestablishment 
   * is not initiated after this time. See 3GPP TS 36.331 7.3.
   */
  Time m_t310;

  /**
   * The 'NoOfOutofSyncs' attribute. This specifies the maximum
   * consecutive out-of-sync indications from lower layers.
   *
   */
  uint8_t m_n310;

  /**
   *  The 'NoOfInSyncs' attribute. This specifies the minimum
   *  consecutive in-sync indications from lower layers.
   *
   */
  uint8_t m_n311;

  /**
   * Time limit (given by m_t310) before the radio link is considered to have failed.
   * Set upon detecting physical layer problems i.e. upon receiving
   * N310 consecutive out-of-sync indications from lower layers. Calling
   * LteUeRrc::RadioLinkFailureDetected() when it expires. Cancelled
   * upon receiving N311 consecutive in-sync indications. Upon
   * expiry, the UE transitions to RRC_IDLE and no RRC connection
   * reestablishment is initiated.
   *
   */
  EventId m_radioLinkFailureDetected;

  uint16_t m_noOfSyncIndications; ///< num of in-sync or out-of-sync indications coming from PHY layer

  bool m_contentionMsgReceived; ///< Set to true if UE contention resolution identity (CriLteControlMessage) is received

  bool m_connectedStateContentionBasedRandomAccess; ///< true if RA occurs in UE RRC connected State

  bool m_realPrach; ///< true if real RACH model is used. Needed to eliminate use of connection resolution message for ideal RACH

  /**
   * Set to true if RRC connection reconfiguration msg was received.
   * If msg is not received, UE is not considered to be uplink out of sync
   * when time alignment timer expires. This is done to avoid errors
   * with respect to SRS transmission (same SRS assignment for multiple UEs)
   * after random access is attempted in connected state.
   */
  bool m_connectionReconfigurationMsgReceived;

  /**
   * The'CellReselection' trace source. Fired upon when UE initiates
   * cell reselection procedure to reselect to a new cell.
   *
   */
  TracedCallback<uint64_t, uint16_t, ReselectionInfo> m_cellReselectionTrace;

  /**
   * The 'UeMobilityStateChanged' trace source. Fired when the mobility of
   * the UE changes in idle state during cell reselection.
   *
   */
  TracedCallback<uint64_t, Time, double, std::string> m_ueMobilityStateChangedTrace;

  EventId m_cellSelectionDueToFailure; ///< initial cell selection is triggered when event expires, event set upon connection failure

  /**
   * True if the random access during RRC connection establishment fails.
   * This is needed to ensure cell reselection is performed atleast once after
   * random access failure to select a better cell instead of triggering repeated
   * random access failures when UL or DL data arrives during bad connectivity. 
   */
  bool m_initialRandomAccessFailed; 

  /**
   * True if the UE resets to IDLE state due to triggering from an
   * external source and not due to any failure of the RRC connection
   * link between UE and eNodeB.
   * 
   * Ex: When running simulations with SUMO, the UE can be deactivated
   * from the simulation script when the UE/car leaves the road network
   */
  bool m_externalTrigger; 

}; // end of class LteUeRrc


} // namespace ns3

#endif // LTE_UE_RRC_H
