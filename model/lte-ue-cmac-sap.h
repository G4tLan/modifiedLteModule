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
 *    (support for RACH realistic model)
 *
 * Modified by Vignesh Babu <ns3-dev@esk.fraunhofer.de>
 *    (support for uplink synchronization;
 *    integrated the RACH realistic model
 *    (taken from Lena-plus(work of Michele Polese)) and also enhanced the module)
 */

#ifndef LTE_UE_CMAC_SAP_H
#define LTE_UE_CMAC_SAP_H

#include <ns3/packet.h>
#include <ns3/ff-mac-common.h>
#include <ns3/eps-bearer.h>
#include <ns3/lte-common.h>
#include <ns3/nstime.h>

namespace ns3 {



class LteMacSapUser;

/**
 * Service Access Point (SAP) offered by the UE MAC to the UE RRC
 *
 * This is the MAC SAP Provider, i.e., the part of the SAP that contains the MAC methods called by the RRC
 */
class LteUeCmacSapProvider
{
public:
  virtual ~LteUeCmacSapProvider ();

  /// RachConfig structure
  struct RachConfig
  {
    uint8_t numberOfRaPreambles; ///< number of RA preambles
    uint8_t preambleTransMax; ///< preamble transmit maximum
    uint8_t raResponseWindowSize; ///< RA response window size

    
    // the following is transmitted with sib2 to 
    //advertise which is the configuration index of the prach
    uint8_t pRachConfigurationIndex;
    int8_t powerRampingStep;
    int8_t preambleInitialReceivedTargetPower;
    uint8_t contentionResolutionTimer;
  };
  
  /** 
   * Configure RACH function 
   *
   * \param rc the RACH config
   */
  virtual void ConfigureRach (RachConfig rc) = 0;

  /** 
   * tell the MAC to start a contention-based random access procedure,
   * e.g., to perform RRC connection establishment 
   * 
   */
  virtual void StartContentionBasedRandomAccessProcedure () = 0;

  /** 
   * tell the MAC to start a non-contention-based random access
   * procedure, e.g., as a consequence of handover
   * 
   * \param rnti
   * \param rapId Random Access Preamble Identifier
   * \param prachMask 
   */
  virtual void StartNonContentionBasedRandomAccessProcedure (uint16_t rnti, uint8_t rapId, uint8_t prachMask) = 0;


  /// LogicalChannelConfig structure
  struct LogicalChannelConfig
  {
    uint8_t priority; ///< priority
    uint16_t prioritizedBitRateKbps; ///< prioritize bit rate Kbps
    uint16_t bucketSizeDurationMs; ///< bucket size duration ms
    uint8_t logicalChannelGroup; ///< logical channel group
  };
  
  /** 
   * add a new Logical Channel (LC) 
   * 
   * \param lcId the ID of the LC
   * \param lcConfig the LC configuration provided by the RRC
   * \param msu the corresponding LteMacSapUser
   */
  virtual void AddLc (uint8_t lcId, LogicalChannelConfig lcConfig, LteMacSapUser* msu) = 0;

  /** 
   * remove an existing LC
   * 
   * \param lcId 
   */
  virtual void RemoveLc (uint8_t lcId) = 0;

  /** 
   * reset the MAC
   * 
   */
  virtual void Reset () = 0;
  
  /**
   *
   * \param rnti the cell-specific UE identifier
   */
  virtual void SetRnti (uint16_t rnti) = 0;
 
   /** 
   * notify that msg3 timer has expired and reset the mac
   * 
   */
  virtual void NotifyConnectionExpired () = 0;

  /** 
   * notify that msg4 has been recvd
   * 
   */
  virtual void NotifyConnectionSuccessful () = 0;  

  /**
   * Set the value of Time alignment timer.
   * Inital value received in SIB2, subsequent values in
   * TAC MAC CE (TacLteControlMessage)
   * 
   */
  virtual void SetTimeAlignmentTimer (uint16_t timeAlignmentTimerCommon) = 0;

};



/**
 * Service Access Point (SAP) offered by the UE MAC to the UE RRC
 *
 * This is the MAC SAP User, i.e., the part of the SAP that contains the RRC methods called by the MAC
 */
class LteUeCmacSapUser
{
public:

  virtual ~LteUeCmacSapUser ();

  /** 
   * 
   * 
   * \param rnti the T-C-RNTI, which will eventually become the C-RNTI after contention resolution
   */
  virtual void SetTemporaryCellRnti (uint16_t rnti) = 0;

  /** 
   * Notify the RRC that the MAC Random Access procedure completed successfully
   * 
   */
 // virtual void NotifyRandomAccessSuccessful () = 0;

  /** 
   * Notify the RRC that the MAC Random Access procedure failed
   * 
   */
  virtual void NotifyRandomAccessFailed () = 0;

  /** 
   * Notify the RRC that the MAC has received a RAR
   * 
   */
  virtual void NotifyRarReceived () = 0;

  /** 
   * Notify the RRC that the MAC has reached timeout for contention resolution
   * 
   */
  virtual void NotifyContentionResolutionTimeout () = 0;

  /**
   * This method is executed when the time alignment timer
   * expires at the MAC layer. The UE is considered to be out of sync
   * only in the RRC CONNECTED_NORMALLY state. If the UE is in any other
   * state, then the UE is considered to be still uplink time aligned
   * even though the timer has expired in order to reduce unwanted errors
   * (such as in case of handover). If UE is out of sync, then the HARQ buffers
   * are cleared, SRS configuration is released and also uplink grants and downlink
   * assignments are cleared. No message or data transmission takes place in this
   * state other than the RACH preamble transmission.
   * 
   *
   * \return true if the UE is considered to be uplink out of sync
   */
    virtual bool NotifyTimeAlignmentTimeout () = 0;

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
    virtual void NotifyEnbTimeAlignmentTimerToStart (Time timeAlignmentTimer, uint16_t rnti) = 0;

    /**
     * Receive notification from MAC that
     * CriLteControlMessage was received
     * 
     */
    virtual void NotifyContentionResolutionMsgReceived()=0;
};




} // namespace ns3


#endif // LTE_UE_CMAC_SAP_H
