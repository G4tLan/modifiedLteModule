/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011, 2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
 * Author: Nicola Baldo <nbaldo@cttc.es>,
 *         Marco Miozzo <mmiozzo@cttc.es>
 *
 * Modified by Vignesh Babu <ns3-dev@esk.fraunhofer.de>
 *    (support for Paging, Radio Link Failure, uplink synchronization)
 */

#ifndef LTE_UE_CPHY_SAP_H
#define LTE_UE_CPHY_SAP_H

#include <stdint.h>
#include <ns3/ptr.h>

#include <ns3/lte-rrc-sap.h>

namespace ns3 {


class LteEnbNetDevice;

/**
 * Service Access Point (SAP) offered by the UE PHY to the UE RRC for control purposes
 *
 * This is the PHY SAP Provider, i.e., the part of the SAP that contains
 * the PHY methods called by the RRC
 */
class LteUeCphySapProvider
{
public:

  /** 
   * destructor
   */
  virtual ~LteUeCphySapProvider ();

  /** 
   * reset the PHY
   * 
   */
  virtual void Reset () = 0;

  /**
   * \brief Tell the PHY entity to listen to PSS from surrounding cells and
   *        measure the RSRP.
   * \param dlEarfcn the downlink carrier frequency (EARFCN) to listen to
   *
   * This function will instruct this PHY instance to listen to the DL channel
   * over the bandwidth of 6 RB at the frequency associated with the given
   * EARFCN.
   *
   * After this, it will start receiving Primary Synchronization Signal (PSS)
   * and periodically returning measurement reports to RRC via
   * LteUeCphySapUser::ReportUeMeasurements function.
   */
  virtual void StartCellSearch (uint32_t dlEarfcn) = 0;

  /**
   * \brief Tell the PHY entity to synchronize with a given eNodeB over the
   *        currently active EARFCN for communication purposes.
   * \param cellId the ID of the eNodeB to synchronize with
   *
   * By synchronizing, the PHY will start receiving various information
   * transmitted by the eNodeB. For instance, when receiving system information,
   * the message will be relayed to RRC via
   * LteUeCphySapUser::RecvMasterInformationBlock and
   * LteUeCphySapUser::RecvSystemInformationBlockType1 functions.
   *
   * Initially, the PHY will be configured to listen to 6 RBs of BCH.
   * LteUeCphySapProvider::SetDlBandwidth can be called afterwards to increase
   * the bandwidth.
   */
  virtual void SynchronizeWithEnb (uint16_t cellId) = 0;

  /**
   * \brief Tell the PHY entity to align to the given EARFCN and synchronize
   *        with a given eNodeB for communication purposes.
   * \param cellId the ID of the eNodeB to synchronize with
   * \param dlEarfcn the downlink carrier frequency (EARFCN)
   *
   * By synchronizing, the PHY will start receiving various information
   * transmitted by the eNodeB. For instance, when receiving system information,
   * the message will be relayed to RRC via
   * LteUeCphySapUser::RecvMasterInformationBlock and
   * LteUeCphySapUser::RecvSystemInformationBlockType1 functions.
   *
   * Initially, the PHY will be configured to listen to 6 RBs of BCH.
   * LteUeCphySapProvider::SetDlBandwidth can be called afterwards to increase
   * the bandwidth.
   */
  virtual void SynchronizeWithEnb (uint16_t cellId, uint32_t dlEarfcn) = 0;

  /**
   * \param dlBandwidth the DL bandwidth in number of PRBs
   */
  virtual void SetDlBandwidth (uint8_t dlBandwidth) = 0;

  /** 
   * \brief Configure uplink (normally done after reception of SIB2)
   * 
   * \param ulEarfcn the uplink carrier frequency (EARFCN)
   * \param ulBandwidth the UL bandwidth in number of PRBs
   */
  virtual void ConfigureUplink (uint32_t ulEarfcn, uint8_t ulBandwidth) = 0;

  /**
   * \brief Configure referenceSignalPower
   *
   * \param referenceSignalPower received from eNB in SIB2
   */
  virtual void ConfigureReferenceSignalPower (int8_t referenceSignalPower) = 0;

  /** 
   * 
   * \param rnti the cell-specific UE identifier
   */
  virtual void SetRnti (uint16_t rnti) = 0;

  /**
   * \param txMode the transmissionMode of the user
   */
  virtual void SetTransmissionMode (uint8_t txMode) = 0;

  /**
   * \param srcCi the SRS configuration index
   */
  virtual void SetSrsConfigurationIndex (uint16_t srcCi) = 0;

  /**
   * \param pa the P_A value
   */
  virtual void SetPa (double pa) = 0;

  /**
   * Reset the UE PHY due to the timeAlignmentTimer expiry.
   * The HARQ buffers are flushed, SRS/PUCCH resources are released
   * and configured uplink grants and downlink assignments are cleared.
   * The UE is considered to be uplink out-of-sync
   * 
   */
  virtual void ResetToOutOfSync () = 0;

  /**
   * Reset the UE to the idle camped state by clearing the
   * appropriate parameters at the PHY
   * 
   */
  virtual void ResetToCamped () = 0;

  /**
   * Configure the parameters for Radio Link Failure detection
   * when RAR is received by UE. RAR reception indicates that the UE
   * is both uplink and downlink synchronized and radio link failure
   * detection procedure can be started.
   * 
   */
  virtual void ConfigureRadioLinkFailureDetection()=0;

  /**
   * When T310 timer is started, it indicates that physical layer
   * problems are detected at the UE and the recovery process is
   * started by checking if the radio frames are in-sync for N311
   * consecutive times.
   * 
   */
  virtual void StartInSnycDetection()=0;

  /**
   * Calculate the paging cycle and paging occasion for the UE
   * according to the paging configuration received in SIB2.
   * See 3GPP 36.304 section 7
   * 
   */
  virtual void ConfigurePaging(LteRrcSap::PcchConfig pagingConfig)=0;

};


/**
 * Service Access Point (SAP) offered by the UE PHY to the UE RRC for control purposes
 *
 * This is the CPHY SAP User, i.e., the part of the SAP that contains the RRC
 * methods called by the PHY
*/
class LteUeCphySapUser
{
public:

  /** 
   * destructor
   */
  virtual ~LteUeCphySapUser ();


  /**
   * Parameters of the ReportUeMeasurements primitive: RSRP [dBm] and RSRQ [dB]
   * See section 5.1.1 and 5.1.3 of TS 36.214
   */
  struct UeMeasurementsElement
  {
    uint16_t m_cellId; ///< cell ID
    double m_rsrp;  ///< [dBm]
    double m_rsrq;  ///< [dB]
  };

  /// UeMeasurementsParameters structure
  struct UeMeasurementsParameters
  {
    std::vector <struct UeMeasurementsElement> m_ueMeasurementsList; ///< UE measurement list
    uint8_t m_componentCarrierId; ///< component carrier ID
  };


  /**
   * \brief Relay an MIB message from the PHY entity to the RRC layer.
   * \param cellId the ID of the eNodeB where the message originates from
   * \param mib the Master Information Block message
   * 
   * This function is typically called after PHY receives an MIB message over
   * the BCH.
   */
  virtual void RecvMasterInformationBlock (uint16_t cellId,
                                           LteRrcSap::MasterInformationBlock mib) = 0;

  /**
   * \brief Relay an SIB1 message from the PHY entity to the RRC layer.
   * \param cellId the ID of the eNodeB where the message originates from
   * \param sib1 the System Information Block Type 1 message
   *
   * This function is typically called after PHY receives an SIB1 message over
   * the BCH.
   */
  virtual void RecvSystemInformationBlockType1 (uint16_t cellId,
                                                LteRrcSap::SystemInformationBlockType1 sib1) = 0;

  /**
   * \brief Send a report of RSRP and RSRQ values perceived from PSS by the PHY
   *        entity (after applying layer-1 filtering) to the RRC layer.
   * \param params the structure containing a vector of cellId, RSRP and RSRQ
   */
  virtual void ReportUeMeasurements (UeMeasurementsParameters params) = 0;

  /**
   * Send an out of sync indication to UE RRC.
   * When the count equals N310, then T310 is started.
   * 
   */
  virtual void NotifyOutOfSync()=0;

  /**
   * Send an in sync indication to UE RRC.
   * When the count equals N311, then T310 is cancelled.
   * 
   */
  virtual void NotifyInSync()=0;

  /**
   * Reset the number of out-of-sync or in-sync indications 
   * sent to the RRC to zero when the out-of-sync or in-sync condition 
   * is not fulfilled during its evaluation respectively. 
   */
  virtual void ResetNumOfSyncIndications()=0;
 
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
  virtual void ReceivePagingMsg(LteRrcSap::RrcPagingMessage msg)=0;

};




/**
 * Template for the implementation of the LteUeCphySapProvider as a member
 * of an owner class of type C to which all methods are forwarded
 * 
 */
template <class C>
class MemberLteUeCphySapProvider : public LteUeCphySapProvider
{
public:
  /**
   * Constructor
   *
   * \param owner the owner class
   */
  MemberLteUeCphySapProvider (C* owner);

  // inherited from LteUeCphySapProvider
  virtual void Reset ();
  virtual void StartCellSearch (uint32_t dlEarfcn);
  virtual void SynchronizeWithEnb (uint16_t cellId);
  virtual void SynchronizeWithEnb (uint16_t cellId, uint32_t dlEarfcn);
  virtual void SetDlBandwidth (uint8_t dlBandwidth);
  virtual void ConfigureUplink (uint32_t ulEarfcn, uint8_t ulBandwidth);
  virtual void ConfigureReferenceSignalPower (int8_t referenceSignalPower);
  virtual void SetRnti (uint16_t rnti);
  virtual void SetTransmissionMode (uint8_t txMode);
  virtual void SetSrsConfigurationIndex (uint16_t srcCi);
  virtual void SetPa (double pa);
  virtual void ResetToOutOfSync ();
  virtual void ResetToCamped ();
  virtual void ConfigureRadioLinkFailureDetection();
  virtual void StartInSnycDetection();
  virtual void ConfigurePaging(LteRrcSap::PcchConfig pagingConfig);

private:
  MemberLteUeCphySapProvider ();
  C* m_owner; ///< the owner class
};

template <class C>
MemberLteUeCphySapProvider<C>::MemberLteUeCphySapProvider (C* owner)
  : m_owner (owner)
{
}

template <class C>
MemberLteUeCphySapProvider<C>::MemberLteUeCphySapProvider ()
{
}

template <class C>
void 
MemberLteUeCphySapProvider<C>::Reset ()
{
  m_owner->DoReset ();
}

template <class C>
void
MemberLteUeCphySapProvider<C>::StartCellSearch (uint32_t dlEarfcn)
{
  m_owner->DoStartCellSearch (dlEarfcn);
}

template <class C>
void
MemberLteUeCphySapProvider<C>::SynchronizeWithEnb (uint16_t cellId)
{
  m_owner->DoSynchronizeWithEnb (cellId);
}

template <class C>
void
MemberLteUeCphySapProvider<C>::SynchronizeWithEnb (uint16_t cellId, uint32_t dlEarfcn)
{
  m_owner->DoSynchronizeWithEnb (cellId, dlEarfcn);
}

template <class C>
void
MemberLteUeCphySapProvider<C>::SetDlBandwidth (uint8_t dlBandwidth)
{
  m_owner->DoSetDlBandwidth (dlBandwidth);
}

template <class C>
void 
MemberLteUeCphySapProvider<C>::ConfigureUplink (uint32_t ulEarfcn, uint8_t ulBandwidth)
{
  m_owner->DoConfigureUplink (ulEarfcn, ulBandwidth);
}

template <class C>
void 
MemberLteUeCphySapProvider<C>::ConfigureReferenceSignalPower (int8_t referenceSignalPower)
{
  m_owner->DoConfigureReferenceSignalPower (referenceSignalPower);
}

template <class C>
void
MemberLteUeCphySapProvider<C>::SetRnti (uint16_t rnti)
{
  m_owner->DoSetRnti (rnti);
}

template <class C>
void 
MemberLteUeCphySapProvider<C>::SetTransmissionMode (uint8_t txMode)
{
  m_owner->DoSetTransmissionMode (txMode);
}

template <class C>
void 
MemberLteUeCphySapProvider<C>::SetSrsConfigurationIndex (uint16_t srcCi)
{
  m_owner->DoSetSrsConfigurationIndex (srcCi);
}

template <class C>
void
MemberLteUeCphySapProvider<C>::SetPa (double pa)
{
  m_owner->DoSetPa (pa);
}

template <class C>
void
MemberLteUeCphySapProvider<C>::ResetToOutOfSync ()
{
  m_owner->DoResetToOutOfSync ();
}

template <class C>
void
MemberLteUeCphySapProvider<C>::ResetToCamped ()
{
  m_owner->DoResetToCamped ();
}

template <class C>
void MemberLteUeCphySapProvider<C>::ConfigureRadioLinkFailureDetection()
{
	m_owner->DoConfigureRadioLinkFailureDetection();
}

template <class C>
void MemberLteUeCphySapProvider<C>::StartInSnycDetection()
{
	m_owner->DoStartInSnycDetection();
}

template <class C>
void MemberLteUeCphySapProvider<C>::ConfigurePaging(LteRrcSap::PcchConfig pagingConfig)
{
  m_owner->DoConfigurePaging(pagingConfig);
}

/**
 * Template for the implementation of the LteUeCphySapUser as a member
 * of an owner class of type C to which all methods are forwarded
 * 
 */
template <class C>
class MemberLteUeCphySapUser : public LteUeCphySapUser
{
public:
  /**
   * Constructor
   *
   * \param owner the owner class
   */
  MemberLteUeCphySapUser (C* owner);

  // methods inherited from LteUeCphySapUser go here
  virtual void RecvMasterInformationBlock (uint16_t cellId,
                                           LteRrcSap::MasterInformationBlock mib);
  virtual void RecvSystemInformationBlockType1 (uint16_t cellId,
                                                LteRrcSap::SystemInformationBlockType1 sib1);
  virtual void ReportUeMeasurements (LteUeCphySapUser::UeMeasurementsParameters params);
  virtual void NotifyOutOfSync();
  virtual void NotifyInSync();
  virtual void ReceivePagingMsg(LteRrcSap::RrcPagingMessage msg);
  virtual void ResetNumOfSyncIndications();


private:
  MemberLteUeCphySapUser ();
  C* m_owner; ///< the owner class
};

template <class C>
MemberLteUeCphySapUser<C>::MemberLteUeCphySapUser (C* owner)
  : m_owner (owner)
{
}

template <class C>
MemberLteUeCphySapUser<C>::MemberLteUeCphySapUser ()
{
}

template <class C> 
void 
MemberLteUeCphySapUser<C>::RecvMasterInformationBlock (uint16_t cellId,
                                                       LteRrcSap::MasterInformationBlock mib)
{
  m_owner->DoRecvMasterInformationBlock (cellId, mib);
}

template <class C>
void
MemberLteUeCphySapUser<C>::RecvSystemInformationBlockType1 (uint16_t cellId,
                                                            LteRrcSap::SystemInformationBlockType1 sib1)
{
  m_owner->DoRecvSystemInformationBlockType1 (cellId, sib1);
}

template <class C>
void
MemberLteUeCphySapUser<C>::ReportUeMeasurements (LteUeCphySapUser::UeMeasurementsParameters params)
{
  m_owner->DoReportUeMeasurements (params);
}

template <class C>
void
MemberLteUeCphySapUser<C>::NotifyOutOfSync()
{
	m_owner->DoNotifyOutOfSync();
}

template <class C>
void
MemberLteUeCphySapUser<C>::NotifyInSync()
{
	m_owner->DoNotifyInSync();
}

template <class C>
void
MemberLteUeCphySapUser<C>::ReceivePagingMsg(LteRrcSap::RrcPagingMessage msg)
{
  m_owner->DoRecvPagingMsg(msg);
}

template <class C>
void
MemberLteUeCphySapUser<C>::ResetNumOfSyncIndications()
{
  m_owner->DoResetNumOfSyncIndications();
}

} // namespace ns3


#endif // LTE_UE_CPHY_SAP_H
