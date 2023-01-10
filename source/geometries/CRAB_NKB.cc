// ----------------------------------------------------------------------------
// nexus | CRAB_NKB.cc
//
// General-purpose cylindric chamber.
//
// The NEXT Collaboration
// ----------------------------------------------------------------------------

#include "CRAB_NKB.h"

#include "PmtR7378A.h"
#include "NextNewKDB.h"
#include "MaterialsList.h"
#include "OpticalMaterialProperties.h"
#include "UniformElectricDriftField.h"
#include "IonizationSD.h"
#include "FactoryBase.h"
#include <G4Box.hh>
#include <G4NistManager.hh>
#include <G4GenericMessenger.hh>
#include <G4Tubs.hh>
#include <G4LogicalVolume.hh>
#include <G4PVPlacement.hh>
#include <G4SDManager.hh>
#include "G4PhysicalVolumeStore.hh"
#include "G4LogicalVolumeStore.hh"
#include "G4SubtractionSolid.hh"

#include <CLHEP/Units/SystemOfUnits.h>
#include <CLHEP/Units/PhysicalConstants.h>


namespace nexus {

  REGISTER_CLASS(CRAB_NKB, GeometryBase)

  using namespace CLHEP;

  CRAB_NKB::CRAB_NKB():
    GeometryBase(), msg_(0)
  {
  }



  CRAB_NKB::~CRAB_NKB()
  {
    delete msg_;
  }



  void CRAB_NKB::Construct()
  {
    //Constructing Lab Space
    G4String lab_name="LAB";
    const G4double Lab_size = 2. *m;


    G4Box * lab_solid_volume = new G4Box(lab_name,Lab_size/2,Lab_size/2,Lab_size/2);
    G4LogicalVolume * lab_logic_volume= new G4LogicalVolume(lab_solid_volume,G4NistManager::Instance()->FindOrBuildMaterial("G4_AIR"),lab_name) ;

    // CHAMBER ///////////////////////////////////////////////////////
    const G4double chamber_diam   =  10. * cm;
    const G4double chamber_length = 40. * cm;
    const G4double chamber_thickn =   0.5 * cm;

    G4Tubs* chamber_solid =
      new G4Tubs("CHAMBER", 0., (chamber_diam/2. + chamber_thickn),
        (chamber_length/2. + chamber_thickn), 0., twopi);

    G4LogicalVolume* chamber_logic =
      new G4LogicalVolume(chamber_solid, materials::Steel(), "CHAMBER");

    this->SetLogicalVolume(chamber_logic);


    // GAS ///////////////////////////////////////////////////////////

    G4Tubs* gas_solid =
      new G4Tubs("GAS", 0., chamber_diam/2., chamber_length/2., 0., twopi);

    G4Material* gxe = materials::GXe(10.*bar);
    gxe->SetMaterialPropertiesTable(opticalprops::GXe(10.*bar, 303));

    G4LogicalVolume* gas_logic = new G4LogicalVolume(gas_solid, gxe, "GAS");

    new G4PVPlacement(0, G4ThreeVector(0.,0.,0.), gas_logic, "GAS",
		      chamber_logic, false, 0, true);


    // ACTIVE ////////////////////////////////////////////////////////

    const G4double active_diam   = 8.5 *cm;
    const G4double active_length = 20. *cm;

    G4Tubs* active_solid =
      new G4Tubs("ACTIVE", 0., active_diam/2., active_length/2., 0, twopi);

    G4LogicalVolume* active_logic =
      new G4LogicalVolume(active_solid, gxe, "ACTIVE");

    new G4PVPlacement(0, G4ThreeVector(0.,0.,0.), active_logic, "ACTIVE",
		      gas_logic, false, 0, true);

    // Define this volume as an ionization sensitive detector
    IonizationSD* sensdet = new IonizationSD("/CRAB_NKB/ACTIVE");
    active_logic->SetSensitiveDetector(sensdet);
    G4SDManager::GetSDMpointer()->AddNewDetector(sensdet);

    // Define an electric drift field for this volume
    UniformElectricDriftField* drift_field = new UniformElectricDriftField();
    drift_field->SetCathodePosition(-active_length/2.);
    drift_field->SetAnodePosition(active_length/2.);
    drift_field->SetDriftVelocity(0.9*mm/microsecond);
    drift_field->SetTransverseDiffusion(1.18*mm/sqrt(cm));
    drift_field->SetLongitudinalDiffusion(.3*mm/sqrt(cm));

    G4Region* drift_region = new G4Region("DRIFT_REGION");
    drift_region->SetUserInformation(drift_field);
    drift_region->AddRootLogicalVolume(active_logic);


    // EL GAP ////////////////////////////////////////////////////////

    const G4double elgap_diam   = active_diam;
    const G4double elgap_length = 0.7 * cm;

    G4Tubs* elgap_solid =
      new G4Tubs("EL_GAP", 0., elgap_diam/2., elgap_length/2., 0, twopi);

    G4LogicalVolume* elgap_logic =
      new G4LogicalVolume(elgap_solid, gxe, "EL_GAP");

    G4double pos_z = active_length/2. + elgap_length/2.;

    new G4PVPlacement(0, G4ThreeVector(0.,0.,pos_z), elgap_logic, "EL_GAP",
		      gas_logic, false, 0, true);

    // Define an EL field for this volume
    UniformElectricDriftField* el_field = new UniformElectricDriftField();
    el_field->SetCathodePosition(active_length/2. + elgap_length);
    el_field->SetAnodePosition(active_length/2.);
    el_field->SetDriftVelocity(0.9*mm/microsecond);
    el_field->SetTransverseDiffusion(1.18*mm/sqrt(cm));
    el_field->SetLongitudinalDiffusion(.3*mm/sqrt(cm));
    el_field->SetLightYield(100./cm);

    G4Region* el_region = new G4Region("EL_REGION");
    el_region->SetUserInformation(el_field);
    el_region->AddRootLogicalVolume(elgap_logic);

    // SOURCE HOLDER /////////////////////////////////////////////////
    const G4double SourceEn_length = 1. * cm;
    const G4double SourceEn_thickn = 2. * mm;
    const G4double SourceEn_holedia = 2. * mm;
    const G4double SourceEn_offset = 1. * cm;
    const G4double SourceEn_diam = 10. * mm;
    const G4double SourceEn_z = 5.35 * cm;

    G4RotationMatrix* rm = new G4RotationMatrix();
    rm->rotateY(90.*deg);

    G4Tubs* SourceHolChamber_solid =
      new G4Tubs("SourceHolChamber", SourceEn_holedia/2, (SourceEn_diam/2. + SourceEn_thickn),(SourceEn_length/2. + SourceEn_thickn),0,twopi);
    G4Tubs* SourceHolChamberBlock_solid =
      new G4Tubs("SourceHolChBlock",0,(SourceEn_holedia/2),( SourceEn_thickn/2), 0.,twopi);


    G4LogicalVolume* SourceHolChamber_logic = 
      new G4LogicalVolume(SourceHolChamber_solid,materials::Steel(), "SourceHolChamber_logic");
    G4LogicalVolume* SourceHolChamberBlock_logic = 
      new G4LogicalVolume(SourceHolChamberBlock_solid,materials::Steel(), "SourceHolChBlock_logic");

    new G4PVPlacement(rm, G4ThreeVector(-SourceEn_offset,0,SourceEn_z), SourceHolChamber_logic, SourceHolChamber_solid->GetName(),gas_logic, false, 0, true);
    
    new G4PVPlacement(rm, G4ThreeVector(-SourceEn_offset-SourceEn_length/2,0,SourceEn_z), SourceHolChamberBlock_logic, SourceHolChamberBlock_solid->GetName(),gas_logic, false, 0, true);


    // PHOTOMULTIPLIER ///////////////////////////////////////////////
    PmtR7378A pmt_geom;

    // Hole Stuff
    const G4double hole_diam = 26. *mm;
    const G4double hole_depth = 4.9 *cm;

    G4Tubs* hole = new G4Tubs("HOLE_TUB", 0., hole_diam/2.,
				   hole_depth, 0., twopi);

    G4SubtractionSolid* pmt_hole =
    new G4SubtractionSolid("HOLE", chamber_solid, hole,
			   0, G4ThreeVector(0., 0., chamber_length/2.+chamber_thickn/2.));
         
    // pmt_geom.SetSensorDepth(0);
    pmt_geom.Construct();

    //Distance of PMT surface from the beginning of EL region
    G4double dist_el = 100.*mm;

    // PMT position in gas
    G4double pmt_length = pmt_geom.Length();
    // G4double posz = active_length/2. + elgap_length/2. + dist_el + pmt_length/2.;
    G4LogicalVolume* pmt_logic = pmt_geom.GetLogicalVolume();
    G4RotationMatrix* rotationMatrix = new G4RotationMatrix();
    rotationMatrix->rotateY(180.*deg);

    pos_z = 22 *cm;

    new G4PVPlacement(rotationMatrix, G4ThreeVector(0., 0., pos_z), pmt_logic,
      "PMT", gas_logic, false, 0, true);


    // // DICE BOARD ////////////////////////////////////////////////////

    // NextNewKDB kdb_geom(5,5);
    // kdb_geom.Construct();

    // G4LogicalVolume* kdb_logic = kdb_geom.GetLogicalVolume();

    // pos_z = active_length/2. + elgap_length + 5.0*mm;

    // new G4PVPlacement(0, G4ThreeVector(0., 0., pos_z), kdb_logic,
    //   "KDB", gas_logic, false, 0, true);


    //// VERTEX GENERATORS
    // lab_gen_ =
    //   new BoxPointSampler(lab_size_ - 1.*m, lab_size_ - 1.*m, lab_size_  - 1.*m, 1.*m,
    //                       G4ThreeVector(0., 0., 0.), 0);

  }



  G4ThreeVector CRAB_NKB::GenerateVertex(const G4String& region) const
  {
   if(!(region=="LAB" || region=="GAS" || region=="ACTIVE" || region=="SourceHolChamber" || region=="PMT_WINDOW")){

      G4Exception("[CRAB]", "GenerateVertex()", FatalException,
                  "Unknown vertex generation region.");
      }
        // return vtx_;
      
    return G4ThreeVector(0.,0.,5.35 * cm);
  }


} // end namespace nexus

