// ----------------------------------------------------------------------------
// nexus | CRAB_NKB.h
//
// General-purpose cylindric chamber.
//
// The NEXT Collaboration
// ----------------------------------------------------------------------------

#ifndef CRAB_NKB_H
#define CRAB_NKB_H

#include "GeometryBase.h"

class G4GenericMessenger;


namespace nexus {

  class CRAB_NKB: public GeometryBase
  {
  public:
    /// Constructor
    CRAB_NKB();
    /// Destructor
    ~CRAB_NKB();

    /// Return vertex within region <region> of the chamber
    virtual G4ThreeVector GenerateVertex(const G4String& region) const;

    virtual void Construct();

  private:
    /// Messenger for the definition of control commands
    G4GenericMessenger* msg_;
  };

} // end namespace nexus

#endif
