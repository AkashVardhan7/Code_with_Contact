// =============================================================================
// Initialize Smarticles from Command Line
// Input from text file
#include "chrono/core/ChStream.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono_irrlicht/ChIrrApp.h"


#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "Skeleton.cpp"

#include <cmath>
#include <sstream>
using namespace chrono;
using namespace chrono::irrlicht;

using namespace irr;
using namespace irr::core;


void AddContainerWall(std::shared_ptr<ChBody> body,
                      std::shared_ptr<ChMaterialSurface> mat,
                      const ChVector<>& size,
                      const ChVector<>& pos,
                      bool visible = true) {
    ChVector<> hsize = 0.5 * size;

    body->GetCollisionModel()->AddBox(mat, hsize.x(), hsize.y(), hsize.z(), pos);
    body->SetNameString("ground 0");
    // body->GetCollisionModel()->SetFamily(1);
    if (visible) {
        auto box = chrono_types::make_shared<ChBoxShape>();
        box->GetBoxGeometry().Size = hsize;
        body->AddAsset(box);
        // body->AddVisualShape(box, ChFrame<>(pos, QUNIT));
    }
}
// ....

void AddContainer(ChSystem* sys) {

// void AddContainer(ChSystemNSC& sys) {
    // The fixed body (5 walls)
    auto fixedBody = chrono_types::make_shared<ChBody>();

    fixedBody->SetMass(1.0);
    fixedBody->SetBodyFixed(true);
    fixedBody->SetPos(ChVector<>());
    fixedBody->SetCollide(true);

    // Contact material for container
    // auto fixed_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    ChContactMethod contact_method = sys->GetContactMethod();

    auto fixed_mat = DefaultContactMaterial(contact_method);


    fixedBody->GetCollisionModel()->ClearModel();
    // ground
   // AddContainerWall(fixedBody, fixed_mat, ChVector<>(5.0, 0.02, 5.0), ChVector<>(0, -0.025, 0), false);
    AddContainerWall(fixedBody, fixed_mat, ChVector<>(5.0, 0.02, 5.0), ChVector<>(0, -0.025, 0), true); // Older Initialization
    //AddContainerWall(fixedBody, fixed_mat, ChVector<>(5.0, 0.02, 5.0), ChVector<>(0, -0.01, 0), true);   // Initialization to ensure Ground is at 0.0
    // lwall
    //AddContainerWall(fixedBody, fixed_mat, ChVector<>(0.02, 5.0, 5.0), ChVector<>(-5.0, 2.0, 0.0), false);
    AddContainerWall(fixedBody, fixed_mat, ChVector<>(0.02, 5.0, 5.0), ChVector<>(-5.0, 2.0, 0.0), true);
    // rwall
   // AddContainerWall(fixedBody, fixed_mat, ChVector<>(0.02, 5.0, 5.0), ChVector<>(5.0, 2.0, 0.0), false);
    AddContainerWall(fixedBody, fixed_mat, ChVector<>(0.02, 5.0, 5.0), ChVector<>(5.0, 2.0, 0.0), true);

    // Back Wall
   // AddContainerWall(fixedBody, fixed_mat, ChVector<>(5.0, 5.0, 0.02), ChVector<>(0, 2.0, -5.0), false);
    AddContainerWall(fixedBody, fixed_mat, ChVector<>(5.0, 5.0, 0.02), ChVector<>(0, 2.0, -5.0), true);

    // Front Wall
   // AddContainerWall(fixedBody, fixed_mat, ChVector<>(5.0, 5.0, 0.02), ChVector<>(0, 2, 5.0), false);
    AddContainerWall(fixedBody, fixed_mat, ChVector<>(5.0, 5.0, 0.02), ChVector<>(0, 2, 5.0), true);

    fixedBody->GetCollisionModel()->BuildModel();


    auto texture = chrono_types::make_shared<ChTexture>();
    texture->SetTextureFilename(GetChronoDataFile("textures/concrete.jpg"));
    fixedBody->AddAsset(texture);

    sys->AddBody(fixedBody);

}


class ContactReporter : public ChContactContainer::ReportContactCallback {
public:
    ContactReporter() {

        // initialize output stream
        txt.stream().setf(std::ios::scientific | std::ios::showpos);
        txt.stream().precision(8);

    }

    bool WriteContactInfo(std::string filename) {
        // Originally
        txt.write_to_file(filename);
        std::cout << "write contact info " << filename << std::endl;


        /*
        int size = txt.stream().str().length();
        if (size > 0) {
            txt.write_to_file(filename);           
            std::cout << "write contact info " << filename << std::endl;
        }
        */

        return true;
    }


private:
    virtual bool OnReportContact(const ChVector<>& pA,  // contact point on body A (global)
        const ChVector<>& pB,  // contact point on body B 
        const ChMatrix33<>& plane_coord,
        const double& distance,
        const double& eff_radius,
        const ChVector<>& cforce,
        const ChVector<>& ctorque,
        ChContactable* modA,
        ChContactable* modB) override {
        // Check if contact involves box1
        // if (modA == m_box1.get() && modB == m_box2.get()) {
        // 
        // If the Force exceeeds a threshol
        if (cforce.Length() > 1e-8) {
            // Writing the name of BodyA and BodyB

           /*
            if (modA->GetPhysicsItem()->GetNameString() == "ground 0" || modB->GetPhysicsItem()->GetNameString() == "ground 0") {
                return true;
            }


            else {

                    txt << modA->GetPhysicsItem()->GetNameString();
                    txt << modB->GetPhysicsItem()->GetNameString();
                    txt << pA; // contact point in the global coordinates
                    txt << pB; // Contact point on Body B in Global Coordinates
                    const ChVector<>& nrm = plane_coord.Get_A_Xaxis();  // Contact Frame for Normal and Friction
                    txt << nrm;
                    ChVector<double> global_force = plane_coord * cforce; // Contact expressed in Global Coordinates
                    txt << global_force;
                    txt << std::endl;
            }
            */
            txt << modA->GetPhysicsItem()->GetNameString();
            txt << modB->GetPhysicsItem()->GetNameString();
            txt << pA; // contact point in the global coordinates
            txt << pB; // Contact point on Body B in Global Coordinates
            const ChVector<>& nrm = plane_coord.Get_A_Xaxis();  // Contact Frame for Normal and Friction
            txt << nrm;
            ChVector<double> global_force = plane_coord * cforce; // Contact expressed in Global Coordinates
            txt << global_force;
            txt << std::endl;

            return true;
        }
    
    utils::CSV_writer txt{ " " };




    int main(int argc, char* argv[]) {
        GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
        SetChronoDataPath(CHRONO_DATA_DIR);


        bool useSMC;
        double S1_pos_x, S1_pos_z, S1_theta, S2_pos_x, S2_pos_z, S2_theta;

        if (argc != 4) {
            std::cout << "useage: ./main <0/1 - NSC/SMC> + <S1 pos_x,y,theta, S2 pos_x,y,theta> + <folder/subfolder> \n";   // Works for Folder and SubFolder
          //  std::cout << "useage: ./main <0/1 - NSC/SMC> + <S1 pos_x,y,theta, S2 pos_x,y,theta> + <folder/> \n";     // Currently Works For Only Folder
            return 0;
        }

        useSMC = atoi(argv[1]);

        // parse initial position 
        std::stringstream ss(argv[2]); // creatr a string stream with size of the second input.
        std::vector<double> initial_pos;
        for (double i; ss >> i;) {
            initial_pos.push_back(i);
            // if next character is a comma ignore it.
            if (ss.peek() == ',')
                ss.ignore();
        }

        S1_pos_x = initial_pos[0];
        S1_pos_z = initial_pos[1];
        S1_theta = initial_pos[2];
        S2_pos_x = initial_pos[3];
        S2_pos_z = initial_pos[4];
        S2_theta = initial_pos[5];

        // parse output folder string, assuming in the format of folder/subfolder
     //  std::string filenames = argv[3];                                                         // Currently Works
      // std::string output_folder_name = filenames.substr(0, filenames.find('/'));               // Currently Works

        // Subfolder
       // std::string subfolder_name = filenames.substr(filenames.find('/')+1, filenames.size());
       // std::cout << "folder " << output_folder_name << "subfolder: " << subfolder_name << std::endl;

        // std::string output_folder_name = argv[3];


        // create folder and subfolder if they do not exist 
       // filesystem::create_directory(filesystem::path(output_folder_name));  // Leave Uncommented for running with Only Folder Name     // Currently Works

        //output_folder_name = output_folder_name + '/' + subfolder_name;
        //output_folder_name = output_folder_name + '\' + subfolder_name;    
       // output_folder_name = output_folder_name + '\\' + subfolder_name;   
       // filesystem::create_directory(filesystem::path(output_folder_name));


        /*
        std::string filenames = "test3/child";
        std::string output_folder_name = filenames.substr(0, filenames.find('/'));
        std::string subfolder_name = filenames.substr(filenames.find('/') + 1, filenames.size());
        filesystem::create_directory(filesystem::path(output_folder_name));
        output_folder_name = output_folder_name + '/' + subfolder_name;
        filesystem::create_directory(filesystem::path(output_folder_name));
        */

        /*
        filesystem::create_directory(filesystem::path("test1"));
        std::cout << "create directory " << filesystem::path("test1") << std::endl;
        filesystem::create_directory(filesystem::path("test1/child/"));
        std::cout << "create directory " << filesystem::path("test1/child/") << std::endl;
        */


        // std::cout << "Current directory = " << filesystem::path(output_folder_name).make_absolute() << std::endl;       // Currently Works



           // For Checking If Subdirectory is being created properly
          // Works Properly if need to store data in directory/subdirectory format.


        std::string filenames = argv[3];
        std::string output_folder_name = filenames;
        if (filesystem::create_subdirectory(filesystem::path(filenames))) {
            std::cout << "..Creating nested subdirectories" << std::endl;
        }
        else {
            std::cout << "..Error creating nested subdirectories" << std::endl;
            return 1;
        }

        std::cout << "Current directory = " << filesystem::path(output_folder_name).make_absolute() << std::endl;


        ChSystem* sys;
        double step_size;

        if (useSMC) {
            ChSystemSMC* my_sys = new ChSystemSMC();
            sys = my_sys;
            // step_size = 1e-4;
            step_size = 5e-5;

        }
        else {
            ChSystemNSC* my_sys = new ChSystemNSC();
            sys = my_sys;
            //step_size = 1e-3;    
           // step_size = 1e-4;
            step_size = 5e-5;
        }



        // double time_end = 16*30;      // 30 Periods worth of data with timestep 1e-4
        //double time_end = 16 * 20 * 2;    // 20 Periods worth of data with timestep 5e-5
       // double time_end = 16 * 300; // 30 Periods Worth of data with timestep 1e-5

        // time for testing
        double time_end = 5.0;

        AddContainer(sys);


        double output_frame_step = 0.01; // write image every 0.01 second


        // ChVector<float> gravity(0, 0, 9.81);
        ChVector<float> gravity(0, -9.81, 0);
        sys->Set_G_acc(gravity);
        // double step_size = 1e-3; // 1e-3 in world.skel which one?

        // actuator (type: servo not velocity)
        // force uppser limit (0, 3.0e-2)
        // force lower limit (0, -3.0e-2)
        // position limit enforced
        // point limit: angle: (0, 1.5708 to 0, -1.5708) 

        // Parameters;
        // location of the belly at -0.02, 0, -0.006
        // theta, oritentation of the belly
        // alpha1 and alpha2
        // body id, make sure it's different for every body
        // 
       // Skeleton skeleton1(ChVector<double>(-0.02f, 0.0f, -0.006f), 0, -CH_C_PI_2, -CH_C_PI_2, 11);
        //Skeleton skeleton1(ChVector<double>(0.00f, 0.0f, 0.0f), 3.001966e+00, -CH_C_PI_2, -CH_C_PI_2, 11);
        //Skeleton skeleton1(ChVector<double>(0.00f, 0.0f, 0.0f), 7.330383e-01, -CH_C_PI_2, -CH_C_PI_2, 11);
        // Skeleton skeleton1(ChVector<double>(0.00f, 0.0f, 0.00f), -1.954769e+00, -CH_C_PI_2, -CH_C_PI_2, 11);
        Skeleton skeleton1(ChVector<double>(S1_pos_x, 0.0f, S1_pos_z), S1_theta, -CH_C_PI_2, -CH_C_PI_2, 11);


        skeleton1.SetContactMethod(sys->GetContactMethod());
        skeleton1.SetSkeletonName("Smarticle1");
        skeleton1.Initialize();

        // Add body skeleton 1 to sys
        skeleton1.AddSkeleton(sys);


        // Test one skeleton first ... then writes API that sets phase shift
        //Skeleton skeleton2(ChVector<double>(0.02, 0.0f, 0.06f), CH_C_PI, -CH_C_PI_2, -CH_C_PI_2, 12);
        //Skeleton skeleton2(ChVector<double>(-2.967361e-02, 0.0f, -6.908784e-02), 6.143559e+00, -CH_C_PI_2, -CH_C_PI_2, 12);
       // Skeleton skeleton2(ChVector<double>(7.199819e-02, 0.0f, 2.167750e-02), 3.874631e+00, -CH_C_PI_2, -CH_C_PI_2, 12);  // Unstable with NSC Stable with SMC
        // Skeleton skeleton2(ChVector<double>(-7.421433e-02, 0.0f, 1.207832e-02), 1.186824e+00, -CH_C_PI_2, -CH_C_PI_2, 12); 
        Skeleton skeleton2(ChVector<double>(S2_pos_x, 0.0f, S2_pos_z), S2_theta, -CH_C_PI_2, -CH_C_PI_2, 12);

        skeleton2.SetContactMethod(sys->GetContactMethod());
        skeleton2.SetSkeletonName("Smarticle2");
        skeleton2.Initialize();
        skeleton2.AddSkeleton(sys);

        // Create the Irrlicht visualization system
        /*
        ChIrrApp application(sys, L"smarticle demo", core::dimension2d<u32>(800, 600));
        // Add camera, lights, logo and sky in Irrlicht scene
        application.AddTypicalLogo();
        application.AddTypicalSky();
        application.AddTypicalLights();
        application.AddTypicalCamera(core::vector3df(0, 0.2, 0));

        // Complete asset specification: convert all assets to Irrlicht
        application.AssetBindAll();
        application.AssetUpdateAll();
        */

        int frame = 0;

        // initialize the csv writer to write the output file
        utils::CSV_writer txt(" ");
        txt.stream().setf(std::ios::scientific | std::ios::showpos);
        txt.stream().precision(8);


        // while (vis->Run()) {
        while (true) {
            // visualization
            // compute dynamics
            sys->DoStepDynamics(step_size);

            /*
            if (frame % int(output_frame_step/step_size) == 0){
                char filename[100];
                sprintf(filename, "img_%01d.jpg", int(frame/100));
                irr::video::IImage* image = application.GetVideoDriver()->createScreenShot();
                application.GetVideoDriver()->writeImageToFile(image, filename);
                image->drop();

            }
            */

            if (frame % 200 == 0) {

                /*
                 application.BeginScene();
                 application.DrawAll();
                 application.EndScene();
                */

                std::cout << "Current Frame Number" << std::endl;
                std::cout << frame << std::endl;
                txt << sys->GetChTime() << skeleton1.GetPos().x() << skeleton1.GetPos().y() << skeleton1.GetPos().z() << skeleton1.GetTheta() << skeleton1.GetAlpha1() << skeleton1.GetAlpha2();


                txt << skeleton2.GetPos().x() << skeleton2.GetPos().y() << skeleton2.GetPos().z() << skeleton2.GetTheta() << skeleton2.GetAlpha1() << skeleton2.GetAlpha2();

                txt << std::endl;


                // Contact Info
                auto creporter = chrono_types::make_shared<ContactReporter>();
                sys->GetContactContainer()->ReportAllContacts(creporter);
                char contact_file_name[500];
                sprintf(contact_file_name, "%s/contact%06d.txt", filenames.c_str(), int(frame / 200));
                creporter->WriteContactInfo(std::string(contact_file_name));

            }

            frame++;



            if (sys->GetChTime() > time_end) {

                break;
            }

        }

        std::string txt_output;
        if (useSMC) {
            txt_output = "SMC_result.txt";
        }
        else {
            txt_output = "NSC_result.txt";

        }
        txt.write_to_file(output_folder_name + '/' + txt_output);
        // txt.write_to_file(output_folder_name + '\\' + txt_output);

        return 0;
    }


