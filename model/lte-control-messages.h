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
 * Author: Marco Miozzo <marco.miozzo@cttc.es>
 *
 * Modified by Michele Polese <michele.polese@gmail.com>
 *    (support for RACH realistic model) 
 *
 *  Modified by Vignesh Babu <ns3-dev@esk.fraunhofer.de>
 *    (support for Paging, uplink synchronization;
 *    integrated the RACH realistic model
 *    (taken from Lena-plus(work of Michele Polese)) and also enhanced the module)
 */

#ifndef LTE_CONTROL_MESSAGES_H
#define LTE_CONTROL_MESSAGES_H

#include <ns3/ptr.h>
#include <ns3/simple-ref-count.h>
#include <ns3/ff-mac-common.h>
#include <ns3/lte-rrc-sap.h>
#include <list>
#include "ns3/vector.h"

namespace ns3 {

class LteNetDevice;


/**
 * \ingroup lte
 *
 * The LteControlMessage provides a basic implementations for
 * control messages (such as PDCCH allocation map, CQI feedbacks)
 * that are exchanged among eNodeB and UEs.
 */
class LteControlMessage : public SimpleRefCount<LteControlMessage>
{
public:
  /**
   * The type of the message
   * NOTE: The messages sent by UE are filtered by the
   *  LteEnbPhy::ReceiveLteControlMessageList in order to remove the ones 
   *  that has been already handoff by the eNB for avoiding propagation of
   *  spurious messages. When new messaged have to been added, consider to
   *  update the switch statement implementing the filtering.
   */
  enum MessageType
  {
    DL_DCI, UL_DCI, // Downlink/Uplink Data Control Indicator
    DL_CQI, UL_CQI, // Downlink/Uplink Channel Quality Indicator
    BSR, // Buffer Status Report
    DL_HARQ, // UL HARQ feedback
    RACH_PREAMBLE, // Random Access Preamble
    RAR, // Random Access Response
    MIB, // Master Information Block
    SIB1, // System Information Block Type 1
    TAC, //Timing Advance Command MAC CE
    CRNTI,//C-RNTI MAC CE
    CRI,//UE contention resolution identity MAC CE
    PAGING
  };

  LteControlMessage (void);
  virtual ~LteControlMessage (void);

  /**
   * \brief Set the type of the message
   * \param type the type of the message
   */
  void SetMessageType (MessageType type);
  /**
   * \brief Get the type of the message
   * \return the type of the message
   */
  MessageType GetMessageType (void);

private:
  MessageType m_type; ///< message type
};


// -----------------------------------------------------------------------

/**
 * \ingroup lte
 * The Downlink Data Control Indicator messages defines the RB allocation for the
 * users in the downlink
 */
class DlDciLteControlMessage : public LteControlMessage
{
public:
  DlDciLteControlMessage (void);
  virtual ~DlDciLteControlMessage (void);

  /**
  * \brief add a DCI into the message
  * \param dci the dci
  */
  void SetDci (DlDciListElement_s dci);

  /**
  * \brief Get dic information
  * \return dci messages
  */
  DlDciListElement_s GetDci (void);

private:
  DlDciListElement_s m_dci; ///< DCI
};


// ---------------------------------------------------------------------------

/**
 * \ingroup lte
 * The Uplink Data Control Indicator messages defines the RB allocation for the
 * users in the uplink
 */
class UlDciLteControlMessage : public LteControlMessage
{
public:
  UlDciLteControlMessage (void);
  virtual ~UlDciLteControlMessage (void);

  /**
  * \brief add a DCI into the message
  * \param dci the dci
  */
  void SetDci (UlDciListElement_s dci);

  /**
  * \brief Get dic information
  * \return dci messages
  */
  UlDciListElement_s GetDci (void);

private:
  UlDciListElement_s m_dci; ///< DCI
};


// ---------------------------------------------------------------------------

/**
 * \ingroup lte
 * The downlink CqiLteControlMessage defines an ideal list of
 * feedback about the channel quality sent by the UE to the eNodeB.
 */
class DlCqiLteControlMessage : public LteControlMessage
{
public:
  DlCqiLteControlMessage (void);
  virtual ~DlCqiLteControlMessage (void);

  /**
  * \brief add a DL-CQI feedback record into the message.
  * \param dlcqi the DL cqi feedback
  */
  void SetDlCqi (CqiListElement_s dlcqi);

  /**
  * \brief Get DL cqi information
  * \return dlcqi messages
  */
  CqiListElement_s GetDlCqi (void);

private:
  CqiListElement_s m_dlCqi; ///< DL CQI
};


// ---------------------------------------------------------------------------

/**
 * \ingroup lte
 * The uplink BsrLteControlMessage defines the specific
 * extension of the CE element for reporting the buffer status report
 */
class BsrLteControlMessage : public LteControlMessage
{
public:
  BsrLteControlMessage (void);
  virtual ~BsrLteControlMessage (void);

  /**
  * \brief add a BSR feedback record into the message.
  * \param bsr the BSR feedback
  */
  void SetBsr (MacCeListElement_s bsr);

  /**
  * \brief Get BSR information
  * \return BSR message
  */
  MacCeListElement_s GetBsr (void);

private:
  MacCeListElement_s m_bsr; ///< BSR

};


// ---------------------------------------------------------------------------

/**
 * \ingroup lte
 * The downlink DlHarqFeedbackLteControlMessage defines the specific
 * messages for transmitting the DL HARQ feedback through PUCCH
 */
class DlHarqFeedbackLteControlMessage : public LteControlMessage
{
public:
  DlHarqFeedbackLteControlMessage (void);
  virtual ~DlHarqFeedbackLteControlMessage (void);

  /**
  * \brief add a DL HARQ feedback record into the message.
  * \param m the DL HARQ feedback
  */
  void SetDlHarqFeedback (DlInfoListElement_s m);

  /**
  * \brief Get DL HARQ information
  * \return DL HARQ message
  */
  DlInfoListElement_s GetDlHarqFeedback (void);

private:
  DlInfoListElement_s m_dlInfoListElement; ///< DL info list element

};


// ---------------------------------------------------------------------------

/**
 * \ingroup lte
 *
 * abstract model for the Random Access Preamble
 */
class RachPreambleLteControlMessage : public LteControlMessage
{
public:
  RachPreambleLteControlMessage (void);
  
  /** 
   * Set the Random Access Preamble Identifier (RAPID), see 3GPP TS 36.321 6.2.2
   *
   * \param rapid the RAPID
   */
  void SetRapId (uint32_t rapid);
  
  /** 
   * 
   * \return the RAPID
   */
  uint32_t GetRapId () const;

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
   * \param time at which the upper layers ask to generate and tx the ra preamble
   */
  void SetStartTime (Time time);

  /**
   * 
   * \return time at which the upper layers ask to generate and tx the ra preamble
   */
  Time GetStartTime (void) const;

  /**
   * internal: Used by LteSpectrumPhy to set the distance between UE and eNB in order to handle 
   * preamble collision at LteEnbMac
   * \param the position
   * 
   */
  void SetPosition (Vector position);

  /**
   * \return the position of the node
   * 
   */
  Vector GetPosition(void) const;


private:
  uint32_t m_rapId; ///< the RAPID
  uint64_t m_imsi;
  Time m_time;
  Vector m_position;
};


// ---------------------------------------------------------------------------

/**
 * \ingroup lte
 *
 * abstract model for the MAC Random Access Response message
 */
class RarLteControlMessage : public LteControlMessage
{
public:
  RarLteControlMessage (void);

  /** 
   * 
   * \param raRnti the RA-RNTI, see 3GPP TS 36.321 5.1.4
   */
  void SetRaRnti (uint16_t raRnti);

  /** 
   * 
   * \return  the RA-RNTI, see 3GPP TS 36.321 5.1.4
   */
  uint16_t GetRaRnti () const;

  /** 
   * 
   * \param backoffIndicator the BI, see 3GPP TS 36.321 5.1.4 and Table 7.2-1
   * 
   */
  void SetBackoffIndicator (uint16_t backoffIndicator);

  /** 
   * 
   * \return  backoffIndicator the BI, see 3GPP TS 36.321 5.1.4 and Table 7.2-1
   * 
   */
  uint16_t GetBackoffIndicator () const;

  /**
   * a MAC RAR and the corresponding RAPID subheader 
   * 
   */
  struct Rar
  {
    uint8_t rapId; ///< RAPID
    BuildRarListElement_s rarPayload; ///< RAR payload
  };

  /** 
   * add a RAR to the MAC PDU, see 3GPP TS 36.321 6.2.3
   * 
   * \param rar the rar
   */
  void AddRar (Rar rar);

  /** 
   * 
   * \return a const iterator to the beginning of the RAR list
   */
  std::list<Rar>::const_iterator RarListBegin () const;
  
  /** 
   * 
   * \return a const iterator to the end of the RAR list
   */
  std::list<Rar>::const_iterator RarListEnd () const;

private:
  std::list<Rar> m_rarList; ///< RAR list
  uint16_t m_raRnti; ///< RA RNTI
  uint16_t m_backoffIndicator;

};


// ---------------------------------------------------------------------------

/**
 * \ingroup lte
 * \brief Abstract model for broadcasting the Master Information Block (MIB)
 *        within the control channel (BCCH).
 *
 * MIB is transmitted by eNodeB RRC and received by UE RRC at every radio frame,
 * i.e., every 10 milliseconds.
 *
 * \sa LteEnbRrc::ConfigureCell, LteEnbPhy::StartFrame,
 *     LteUeRrc::DoRecvMasterInformationBlock
 */
class MibLteControlMessage : public LteControlMessage
{
public:
  /**
   * \brief Create a new instance of MIB control message.
   */
  MibLteControlMessage (void);

  /**
   * \brief Replace the MIB content of this control message.
   * \param mib the desired MIB content
   */
  void SetMib (LteRrcSap::MasterInformationBlock mib);

  /**
   * \brief Retrieve the MIB content from this control message.
   * \return the current MIB content that this control message holds
   */
  LteRrcSap::MasterInformationBlock GetMib () const;

private:
  LteRrcSap::MasterInformationBlock m_mib; ///< MIB

}; // end of class MibLteControlMessage


// ---------------------------------------------------------------------------

/**
 * \ingroup lte
 * \brief Abstract model for broadcasting the System Information Block Type 1
 *        (SIB1) within the control channel (BCCH).
 *
 * SIB1 is transmitted by eNodeB RRC and received by UE RRC at the 6th subframe
 * of every odd-numbered radio frame, i.e., every 20 milliseconds.
 *
 * \sa LteEnbRrc::SetSystemInformationBlockType1, LteEnbPhy::StartSubFrame,
 *     LteUeRrc::DoRecvSystemInformationBlockType1
 */
class Sib1LteControlMessage : public LteControlMessage
{
public:
  /**
   * \brief Create a new instance of SIB1 control message.
   */
  Sib1LteControlMessage (void);

  /**
   * \brief Replace the SIB1 content of this control message.
   * \param sib1 the desired SIB1 content
   */
  void SetSib1 (LteRrcSap::SystemInformationBlockType1 sib1);

  /**
   * \brief Retrieve the SIB1 content from this control message.
   * \return the current SIB1 content that this control message holds
   */
  LteRrcSap::SystemInformationBlockType1 GetSib1 () const;

private:
  LteRrcSap::SystemInformationBlockType1 m_sib1; ///< SIB1

}; // end of class Sib1LteControlMessage

/**
 * \ingroup lte
 * The downlink TacLteControlMessage defines the specific
 * extension of the MacCeListElement_s for sending the
 * timing advance commands.
 * See 3GPP TS 36.321 6.1.3.5.
 * 
 */
class TacLteControlMessage : public LteControlMessage
{
public:
  TacLteControlMessage (void);
  virtual ~TacLteControlMessage (void);

  /**
  * \brief add a Timing advance command MAC CE into the message.
  * \param tac the Timing advance command MAC CE
  */
  void SetTac (MacCeListElement_s tac);

  /**
  * \brief Get Timing advance command MAC CE
  * \return Timing advance command MAC CE
  */
  MacCeListElement_s GetTac (void) const;

  /**
   * \brief Add the timeAlignmentTimer value to the msg
   * \param timeAlignmentTimer the timeAlignmentTimer value
   */
  void SetTimeAlignmentTimer (uint16_t timeAlignmentTimer);

  /**
   * \brief Get TimeAlignmentTimer value
   * \return the timeAlignmentTimer value
   */
  uint16_t GetTimeAlignmentTimer() const;

private:
  MacCeListElement_s m_tac; ///< Tac
  uint16_t m_timeAlignmentTimer; //(values: 500, 750, 1280, 1920, 2560, 5120, 10240 or infinity sf)

};

/**
 * \ingroup lte
 * The downlink CRntiLteControlMessage defines the specific
 * extension of the MacCeListElement_s for sending CRNTI MAC CE.
 * The msg is sent during RACH carried out for UE in uplink
 * non-synchronized RRC Connected State when uplink data arrives.
 * See 3GPP TS 36.321 6.1.3.2.
 * 
 */
class CRntiLteControlMessage : public LteControlMessage
{
public:
  CRntiLteControlMessage (void);
  virtual ~CRntiLteControlMessage (void);

  /**
  * \brief add the C-RNTI MAC CE into the message.
  * \param crnti the C-RNTI MAC CE
  */
  void SetCRnti (MacCeListElement_s crnti);

  /**
  * \brief Get CRnti MAC CE
  * \return CRnti MAC CE
  */
  MacCeListElement_s GetCRnti (void) const;

  /**
    * \brief add the temp C-RNTI into the message
    * assigned to the UE during RACH. Used to identify the UEs
    * from which this msg was received at the eNodeB
    * \param tempRnti the temp C-RNTI assigned to one or more UEs
    */
    void SetTempRnti (uint16_t tempRnti);

    /**
    * \brief Get the temp C-RNTI
    * \return the temp C-RNTI
    */
    uint16_t GetTempRnti (void) const;

private:
  MacCeListElement_s m_crnti; ///< C-RNTI MAC CE
  uint16_t m_tempRnti;///<temp C-RNTI

};

/**
 * \ingroup lte
 * The downlink CriLteControlMessage defines the specific
 * messages for sending
 * UE Contention Resolution Identity MAC CE to resolve contention.
 * See 3GPP TS 36.321 6.1.3.4.
 * 
 */
class CriLteControlMessage : public LteControlMessage
{
public:
	CriLteControlMessage (void);
  virtual ~CriLteControlMessage (void);

  /**
  * \brief add the UE Contention Resolution Identity into the message.
  * \param ueContentionResolutionIdentity the UE identity to resolve contention (here, the UE IMSI)
  */
  void SetUeContentionResolutionIdentity (uint64_t ueContentionResolutionIdentity);

  /**
  * \brief Get the UE Contention Resolution Identity
  * \return the UE Contention Resolution Identity
  */
  uint64_t GetUeContentionResolutionIdentity (void) const;

  /**
   * \brief add the RNTI assigned to the UE into the message
   * \param rnti the RNTI assigned to the UE
   */
  void SetRnti (uint16_t rnti);

  /**
  * \brief Get the RNTI of the UE
  * \return the RNTI of the UE
  */
  uint16_t GetRnti () const;

private:
  uint64_t m_ueContentionResolutionIdentity; //IMSI of the UE
  uint16_t m_rnti;///< C-RNTI
};

// ---------------------------------------------------------------------------

/**
 * \ingroup lte
 * The downlink PagingLteControlMessage defines the specific
 * messages for sending paging messages
 * See 3GPP TS 36.331 6.2.2.
 * 
 */
class PagingLteControlMessage : public LteControlMessage
{
public:
  /**
   * \brief Create a new instance of RrcPagingMessage control message.
   */
  PagingLteControlMessage (void);

  /**
   * \brief Replace the RrcPagingMessage content of this control message.
   * \param mib the desired RrcPagingMessage content
   */
  void SetRrcPagingMsg (LteRrcSap::RrcPagingMessage msg);

  /**
   * \brief Retrieve the RrcPagingMessage content from this control message.
   * \return the current RrcPagingMessage content that this control message holds
   */
  LteRrcSap::RrcPagingMessage GetRrcPagingMsg () const;

  uint16_t GetPRnti() const;

private:
  LteRrcSap::RrcPagingMessage m_pagingMsg; ///< RrcPagingMessage
  uint16_t m_pRnti;

}; // end of class PagingLteControlMessage


} // namespace ns3

#endif  // LTE_CONTROL_MESSAGES_H
