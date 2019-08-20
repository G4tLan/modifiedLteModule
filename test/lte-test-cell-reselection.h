/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
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
 * Author: Vignesh Babu <ns3-dev@esk.fraunhofer.de>
 */

#ifndef LTE_TEST_CELL_RESELECTION_H
#define LTE_TEST_CELL_RESELECTION_H


#include <ns3/test.h>
#include <ns3/nstime.h>
#include <ns3/node-container.h>
#include <ns3/vector.h>
#include <ns3/lte-ue-rrc.h>
#include <vector>


namespace ns3 {

class LteUeNetDevice;

}

using namespace ns3;


/**
 * \brief Test suite for executing the cell reselection test cases
 *
 * \sa ns3::LteCellReselectionTestCase
 */
class LteCellReselectionTestSuite : public TestSuite
{
public:
  LteCellReselectionTestSuite ();
};

/**
 * \ingroup lte
 *
 * \brief Testing the cell reselection procedure by UE at IDLE state
 */
class LteCellReselectionTestCase : public TestCase
{
public:

  /**
   * A set of parameters which has to verified after reselection is performed
   */
  struct ReselectionResults
  {
    double expectedQHyst;///< the expected value of QHyst based on the mobility state
    Time expectedTimerDuration;///< the expected cell reselection timer duration based on the mobility state
    Time checkPoint;///< the simulation time at which reselection results are verified
    uint16_t cellIdBeforeLastReselection;///< the cell id of the cell that UE is expected to be last camped on
    uint16_t currentReselectedCellId;   ///< the cell id of the cell that the UE is expected to be currently camped on
    std::string expectedMobilitystate; ///< the mobility state of the UE that is expected after cell reselection is performed
  };

  /**
   * \brief Creates an instance of the initial cell reselection test case.
   * \param name name of this test
   * \param isIdealRrc if true, simulation uses Ideal RRC protocol, otherwise
   *                   simulation uses Real RRC protocol
   * \param velocity the velocity of the UE
   * \param reselectionResults set of parameters which has to verified after reselection is performed
   */
  LteCellReselectionTestCase (std::string name, bool isIdealRrc,
                              Vector velocity, ReselectionResults reselectionResults);

  virtual ~LteCellReselectionTestCase ();

private:
  /**
   * \brief Setup the simulation according to the configuration set by the
   *        class constructor, run it, and verify the result.
   */
  virtual void DoRun ();

  /**
   * \brief Verifies if the cell reselection was performed
   * between the expected cells and also verify if the parameter
   * values are of correct mobility state
   * \param ueDev the UE device
   * \param reselectionResults set of parameters to be verified
   */
  void CheckPoint (Ptr<LteUeNetDevice> ueDev, ReselectionResults reselectionResults);

  /**
   * \brief State transition callback function
   * \param context the context string
   * \param imsi the IMSI
   * \param cellId the cell ID
   * \param rnti the RNTI
   * \param oldState the old state
   * \param newState the new state
   */
  void StateTransitionCallback (std::string context, uint64_t imsi,
                                uint16_t cellId, uint16_t rnti,
                                LteUeRrc::State oldState, LteUeRrc::State newState);

  /**
   * \brief Connection established callback function
   * \param context the context string
   * \param imsi the IMSI
   * \param cellId the cell ID
   * \param rnti the RNTI
   */
  void ConnectionEstablishedCallback (std::string context, uint64_t imsi,
                                      uint16_t cellId, uint16_t rnti);

  /**
   * \brief Cell Reselection callback function
   * \param context the context string
   * \param imsi the IMSI
   * \param rnti the RNTI
   * \param reselectionInfo reselection parameters
   */
  void CellReselectionCallback (std::string context, uint64_t imsi, uint16_t rnti,
                                LteUeRrc::ReselectionInfo reselectionInfo);

  /**
   * \brief Ue Mobility State Changed callback function
   * \param context the context string
   * \param imsi the IMSI
   * \param reselectionTimerDuration reselectionTimer Duration
   * \param qHyst the hystersis value
   * \param type type of mobility
   */
  void UeMobilityStateChangedCallback (std::string context, uint64_t imsi, Time reselectionTimerDuration,
                                       double qHyst, std::string type);

  bool m_isIdealRrc; ///< whether the LTE is configured to use ideal RRC
  Vector m_velocity;///< the velocity of the UE
  ReselectionResults m_reselectionResults; ///< the set of reselection parameters to be verified

  /// The current UE RRC state.
  LteUeRrc::State m_lastState;

  uint16_t m_lastCampedOnCellId;///< the cell id of the cell that UE was last camped on
  double m_qHyst; ///< the value of QHyst for the UE
  Time m_reselectionTimerDuration; ///< the reselection timer duration for the UE
  std::string m_mobilityState; ///< the mobility state of the UE during idle mode

}; // end of class LteCellReselectionTestCase

#endif /* LTE_TEST_CELL_RESELECTION_H */
