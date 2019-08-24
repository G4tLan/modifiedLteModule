/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
 * Author: Nicola Baldo <nbaldo@cttc.cat>
 *
 * Modified by Vignesh Babu <ns3-dev@esk.fraunhofer.de> (support for Paging)
 */

#ifndef EPC_S11_SAP_H
#define EPC_S11_SAP_H

#include <ns3/address.h>
#include <ns3/ptr.h>
#include <ns3/object.h>
#include <ns3/eps-bearer.h>
#include <ns3/epc-tft.h>
#include <list>

namespace ns3 {

/**
 * EpcS11Sap
 */
class EpcS11Sap
{
public:

  virtual ~EpcS11Sap ();

  /**
   * GTPC message
   */
  struct GtpcMessage
  {
    uint32_t teid; ///< TEID
  };

  /**
   * Fully-qualified TEID, see 3GPP TS 29.274 section 8.22
   * 
   */
  struct Fteid 
  {
    uint32_t teid; ///< TEID
    Ipv4Address address; ///< IP address
  };

  /**
   * TS 29.274 8.21  User Location Information (ULI)
   * 
   */
  struct Uli
  {
    uint16_t gci; ///< GCI
  };
  
 
};

/**
 * \ingroup lte
 *
 * MME side of the S11 Service Access Point (SAP), provides the MME
 * methods to be called when an S11 message is received by the MME. 
 */
class EpcS11SapMme : public EpcS11Sap
{
public:
  
 /**
   * 3GPP TS 29.274 version 8.3.1 Release 8 section 8.28
   * 
   */
  struct BearerContextCreated
  {

    EpcS11Sap::Fteid sgwFteid; ///< EPC FTEID
    uint8_t epsBearerId; ///< EPS bearer ID
    EpsBearer bearerLevelQos; ///< EPS bearer
    Ptr<EpcTft> tft; ///< traffic flow template
  };


  /**     
   * Create Session Response message, see 3GPP TS 29.274 7.2.2
   */
  struct CreateSessionResponseMessage : public GtpcMessage
  {
    std::list<BearerContextCreated> bearerContextsCreated; ///< bearer contexts created
  };


  /** 
   * send a Create Session Response message
   * 
   * \param msg the message
   */
  virtual void CreateSessionResponse (CreateSessionResponseMessage msg) = 0;

  /**
   * Bearer Context Removed structure
   */
  struct BearerContextRemoved
  {
    uint8_t epsBearerId; ///< EPS bearer ID
  };

  /**
   * Delete Bearer Request message, see 3GPP TS 29.274 Release 9 V9.3.0 section 7.2.9.2
   */
  struct DeleteBearerRequestMessage : public GtpcMessage
  {
    std::list<BearerContextRemoved> bearerContextsRemoved; ///< list of bearer context removed
  };

  /**
    * \brief As per 3GPP TS 29.274 Release 9 V9.3.0, a Delete Bearer Request message shall be sent on the S11 interface by PGW to SGW and from SGW to MME
    * \param msg the message
    */
  virtual void DeleteBearerRequest (DeleteBearerRequestMessage msg) = 0;




  /**     
   * Modify Bearer Response message, see 3GPP TS 29.274 7.2.7
   */
  struct ModifyBearerResponseMessage : public GtpcMessage
  {
    /// Cause enumeration
    enum Cause {
      REQUEST_ACCEPTED = 0,
      REQUEST_ACCEPTED_PARTIALLY,
      REQUEST_REJECTED,
      CONTEXT_NOT_FOUND
    } cause; ///< the cause
  };

  /** 
   * send a Modify Bearer Response message
   * 
   * \param msg the message
   */
  virtual void ModifyBearerResponse (ModifyBearerResponseMessage msg) = 0;

  /**
   * Downlink Data Notification message, see 3GPP TS 29.274 7.2.11
   * 
   */
  struct DownlinkDataNotificationMessage
  {
    uint64_t imsi; //Here, IMSI is used instead of Eps bearer ID
  };

  /**
   * Send a Downlink Data Notification message to MME to start the paging procedure
   * 
   *
   * \param msg Downlink Data Notification message
   */
  virtual void DownlinkDataNotification(DownlinkDataNotificationMessage msg)=0;
};

/**
 * \ingroup lte
 *
 * SGW side of the S11 Service Access Point (SAP), provides the SGW
 * methods to be called when an S11 message is received by the SGW. 
 */
class EpcS11SapSgw : public EpcS11Sap
{
public:

  /// BearerContextToBeCreated structure
  struct BearerContextToBeCreated
  {    
    EpcS11Sap::Fteid sgwFteid; ///< FTEID
    uint8_t epsBearerId; ///< EPS bearer ID
    EpsBearer bearerLevelQos; ///< bearer QOS level
    Ptr<EpcTft> tft; ///< traffic flow template
  };

  
  /**     
   * Create Session Request message, see 3GPP TS 29.274 7.2.1
   */
  struct CreateSessionRequestMessage : public GtpcMessage
  {
    uint64_t imsi; ///< IMSI
    Uli uli; ///< ULI
    std::list<BearerContextToBeCreated> bearerContextsToBeCreated; ///< list of bearer contexts to be created    
  };

  /** 
   * send a Create Session Request message
   * 
   * \param msg the message
   */
  virtual void CreateSessionRequest (CreateSessionRequestMessage msg) = 0;

  /// BearerContextToBeCreated structure
  struct BearerContextToBeRemoved
  {
    uint8_t epsBearerId; ///< EPS bearer ID
  };

  /**
   * Delete Bearer Command message, see 3GPP TS 29.274 Release 9 V9.3.0 section 7.2.17.1
   */
  struct DeleteBearerCommandMessage : public GtpcMessage
  {
    std::list<BearerContextToBeRemoved> bearerContextsToBeRemoved;  ///< list of bearer contexts to be removed    
  };

  /**
    * \brief As per 3GPP TS 29.274 Release 9 V9.3.0, a Delete Bearer Command message shall be sent on the S11 interface by the MME to the SGW
    * \param msg the DeleteBearerCommandMessage
    */
  virtual void DeleteBearerCommand (DeleteBearerCommandMessage msg) = 0;

  /// BearerContextRemovedSgwPgw structure
  struct BearerContextRemovedSgwPgw
  {
    uint8_t epsBearerId; ///< EPS bearer ID
  };

  /**
   * Delete Bearer Response message, see 3GPP TS 29.274 Release 9 V9.3.0 section 7.2.10.2
   */
  struct DeleteBearerResponseMessage : public GtpcMessage
  {
    std::list<BearerContextRemovedSgwPgw> bearerContextsRemoved;   ///< list of bearer contexts removed    
  };

  /**
    * \brief As per 3GPP TS 29.274 Release 9 V9.3.0, a Delete Bearer Command message shall be sent on the S11 interface by the MME to the SGW
    * \param msg the message
    */
  virtual void DeleteBearerResponse (DeleteBearerResponseMessage msg) = 0;

  /**     
   * Modify Bearer Request message, see 3GPP TS 29.274 7.2.7
   */
  struct ModifyBearerRequestMessage : public GtpcMessage
  {
    Uli uli; ///< ULI
  };

  /** 
   * send a Modify Bearer Request message
   * 
   * \param msg the message
   */
  virtual void ModifyBearerRequest (ModifyBearerRequestMessage msg) = 0;

  /**
   * Send the buffered downlink packets to UE once RRC connection is established
   * 
   *
   * \param imsi the unique identifier of the UE
   */
  virtual void SendBufferedDlPackets(uint64_t imsi)=0;

  /**
   * Discard the buffered downlink packets for an UE upon paging failure
   * 
   *
   * \param imsi the unique identifier of the UE
   */
  virtual void DiscardBufferedDlPackets(uint64_t imsi)=0;
};







/**
 * Template for the implementation of the EpcS11SapMme as a member
 * of an owner class of type C to which all methods are forwarded
 * 
 */
template <class C>
class MemberEpcS11SapMme : public EpcS11SapMme
{
public:
  /**
   * Constructor
   *
   * \param owner the owner class
   */
  MemberEpcS11SapMme (C* owner);

  // inherited from EpcS11SapMme
  virtual void CreateSessionResponse (CreateSessionResponseMessage msg);
  virtual void ModifyBearerResponse (ModifyBearerResponseMessage msg);
  virtual void DeleteBearerRequest (DeleteBearerRequestMessage msg);

  /**
   * Send a Downlink Data Notification message to MME to start the paging procedure
   * 
   *
   * \param msg Downlink Data Notification message
   */
  virtual void DownlinkDataNotification(DownlinkDataNotificationMessage msg);

private:
  MemberEpcS11SapMme ();
  C* m_owner; ///< owner class
};

/**
 * Constructor
 *
 * \param owner the owner class
 */
template <class C>
MemberEpcS11SapMme<C>::MemberEpcS11SapMme (C* owner)
  : m_owner (owner)
{
}

template <class C>
MemberEpcS11SapMme<C>::MemberEpcS11SapMme ()
{
}

template <class C>
void MemberEpcS11SapMme<C>::CreateSessionResponse (CreateSessionResponseMessage msg)
{
  m_owner->DoCreateSessionResponse (msg);
}

template <class C>
void MemberEpcS11SapMme<C>::DeleteBearerRequest (DeleteBearerRequestMessage msg)
{
  m_owner->DoDeleteBearerRequest (msg);
}

template <class C>
void MemberEpcS11SapMme<C>::ModifyBearerResponse (ModifyBearerResponseMessage msg)
{
  m_owner->DoModifyBearerResponse (msg);
}

template <class C>
void MemberEpcS11SapMme<C>::DownlinkDataNotification(DownlinkDataNotificationMessage msg)
{
  m_owner->DoDownlinkDataNotification (msg);
}



/**
 * Template for the implementation of the EpcS11SapSgw as a member
 * of an owner class of type C to which all methods are forwarded
 * 
 */
template <class C>
class MemberEpcS11SapSgw : public EpcS11SapSgw
{
public:
/**
 * Constructor
 *
 * \param owner the owner class
 */
  MemberEpcS11SapSgw (C* owner);

  // inherited from EpcS11SapSgw
  virtual void CreateSessionRequest (CreateSessionRequestMessage msg);
  virtual void ModifyBearerRequest (ModifyBearerRequestMessage msg);
  virtual void DeleteBearerCommand (DeleteBearerCommandMessage msg);
  virtual void DeleteBearerResponse (DeleteBearerResponseMessage msg);
  /**
   * Send the buffered downlink packets to UE once RRC connection is established
   * 
   *
   * \param imsi the unique identifier of the UE
   */
  virtual void SendBufferedDlPackets(uint64_t imsi);
  /**
   * Discard the buffered downlink packets for an UE upon paging failure
   * 
   *
   * \param imsi the unique identifier of the UE
   */
  virtual void DiscardBufferedDlPackets(uint64_t imsi);

private:
  MemberEpcS11SapSgw ();
  C* m_owner; ///< owner class
};

/**
 * Constructor
 *
 * \param owner the owner class
 */
template <class C>
MemberEpcS11SapSgw<C>::MemberEpcS11SapSgw (C* owner)
  : m_owner (owner)
{
}

template <class C>
MemberEpcS11SapSgw<C>::MemberEpcS11SapSgw ()
{
}

template <class C>
void MemberEpcS11SapSgw<C>::CreateSessionRequest (CreateSessionRequestMessage msg)
{
  m_owner->DoCreateSessionRequest (msg);
}

template <class C>
void MemberEpcS11SapSgw<C>::ModifyBearerRequest (ModifyBearerRequestMessage msg)
{
  m_owner->DoModifyBearerRequest (msg);
}

template <class C>
void MemberEpcS11SapSgw<C>::DeleteBearerCommand (DeleteBearerCommandMessage msg)
{
  m_owner->DoDeleteBearerCommand (msg);
}

template <class C>
void MemberEpcS11SapSgw<C>::DeleteBearerResponse (DeleteBearerResponseMessage msg)
{
  m_owner->DoDeleteBearerResponse (msg);
}

template <class C>
void MemberEpcS11SapSgw<C>::SendBufferedDlPackets(uint64_t imsi)
{
  m_owner->DoSendBufferedDlPackets(imsi);
}

template <class C>
void MemberEpcS11SapSgw<C>::DiscardBufferedDlPackets(uint64_t imsi)
{
  m_owner->DoDiscardBufferedDlPackets(imsi);
}
} //namespace ns3

#endif /* EPC_S11_SAP_H */

