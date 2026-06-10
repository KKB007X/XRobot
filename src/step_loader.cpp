#include "xrobot/step_loader.hpp"
#include "xrobot/model.hpp"
#include "xrobot/part.hpp"
#include "xrobot/joint.hpp"

#include <iostream>
#include <filesystem>

#include <STEPCAFControl_Reader.hxx>

#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <TDocStd_Document.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_Name.hxx>

#include <TopoDS_Shape.hxx>
#include <TopLoc_Location.hxx>
#include <gp_Trsf.hxx>
#include <gp_XYZ.hxx>
#include <gp_Quaternion.hxx>

#include <BRepMesh_IncrementalMesh.hxx>
#include <StlAPI_Writer.hxx>

#include <string>

#include <TCollection_AsciiString.hxx>

#include <STEPControl_Reader.hxx>
#include <Interface_Static.hxx>

bool xrobot::StepLoader::Load(const char *filename, xrobot::Model& model)
{
    std::cout
        << "Loading STEP: "
        << filename
        << std::endl;

    if (!std::filesystem::exists(filename))
    {
        std::cout
            << "STEP file not found"
            << std::endl;

        return false;
    }

    Handle(XCAFApp_Application) app = XCAFApp_Application::GetApplication();

    Handle(TDocStd_Document) doc;

    app->NewDocument("MDTV-XCAF",doc);

    STEPCAFControl_Reader reader;

    IFSelect_ReturnStatus status = reader.ReadFile(filename);

    std::cout
    << "STEP units: "
    << Interface_Static::CVal("xstep.cascade.unit")
    << std::endl;

    std::unordered_map<std::string, double> unitScale =
    {
        {"MM", 0.001},
        {"CM", 0.01},
        {"M", 1.0},
        {"IN", 0.0254},
        {"FT", 0.3048}
    };

    std::string unit = Interface_Static::CVal("xstep.cascade.unit");

    double scale = 1.0;
    auto it = unitScale.find(unit);

    if (it != unitScale.end())
    {
        scale = it->second;
    }

    model.scale = scale;

    if (status != IFSelect_RetDone)
    {
        std::cout
            << "Failed to load STEP file"
            << std::endl;

        return false;
    }

    if (!reader.Transfer(doc))
    {
        std::cout
            << "Failed to transfer STEP data"
            << std::endl;

        return false;
    }

    StlAPI_Writer writer;
    std::filesystem::create_directories("meshes");

    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());

    TDF_LabelSequence labels;

    shapeTool->GetFreeShapes(labels);

    gp_Trsf baseTransform;
    std::cout
    << baseTransform.Form()
    << std::endl;
    bool foundBase = false;

    std::cout
        << "Top level shapes: "
        << labels.Length()
        << std::endl;

    for (Standard_Integer i = 1;
        i <= labels.Length();
        i++)
    {
        TDF_Label label = labels.Value(i);
        Handle(TDataStd_Name) name;

        if (
            label.FindAttribute(TDataStd_Name::GetID(),name))
        {
            TCollection_AsciiString asciiName(name->Get());

            std::cout
                << "Part: "
                << asciiName.ToCString()
                << std::endl;
            
            TDF_LabelSequence components;

            shapeTool->GetComponents(label,components);

            std::cout
                << "Components: "
                << components.Length()
                << std::endl;
            
            for (Standard_Integer j = 1;
                j <= components.Length();
                j++)
            {
                TDF_Label component =
                    components.Value(j);

                TDF_Label referredShape;

                if (!shapeTool->GetReferredShape(
                        component,
                        referredShape))
                {
                    continue;
                }

                Handle(TDataStd_Name) shapeName;

                if (!referredShape.FindAttribute(
                        TDataStd_Name::GetID(),
                        shapeName))
                {
                    continue;
                }

                TCollection_AsciiString asciiName(
                    shapeName->Get());

                std::string partName =
                    asciiName.ToCString();

                if (partName == "Base_link")
                {
                    TopoDS_Shape shape =
                        shapeTool->GetShape(component);

                    baseTransform =
                        shape.Location().Transformation();

                    foundBase = true;

                    std::cout
                        << "Found base link: "
                        << partName
                        << std::endl;

                    break;
                }
            }
            if (!foundBase)
            {
                std::cout
                    << "Base link not found in STEP"
                    << std::endl;
            }
            for (Standard_Integer j = 1;
                j <= components.Length();
                j++)
            {
                TDF_Label component = components.Value(j);

                TDF_Label referredShape;

                if (
                    shapeTool->GetReferredShape(component, referredShape))
                {
                    Handle(TDataStd_Name) shapeName;

                    if (referredShape.FindAttribute(TDataStd_Name::GetID(), shapeName))
                    {
                        TCollection_AsciiString asciiName(shapeName->Get());
                        std::string partName = asciiName.ToCString();

                        std::cout
                            << "  "
                            << partName;
                        xrobot::Part* part = model.FindPart(partName);

                        if (!part){
                            std::cout << " : no matches found" << std::endl;
                        }
                        else{
                            std::cout << " : match found" << std::endl;

                            TopoDS_Shape shape = shapeTool->GetShape(component);

                            TopoDS_Shape localShape = shape.Located(TopLoc_Location());

                            TopLoc_Location location = shape.Location();
                            gp_Trsf worldTransform = location.Transformation();

                            gp_Trsf transform = baseTransform.Inverted() * worldTransform;

                            gp_XYZ translation = transform.TranslationPart();
                            gp_Quaternion rotation = transform.GetRotation();

                            part->initialPx = translation.X()*scale;
                            part->initialPy = translation.Y()*scale;
                            part->initialPz = translation.Z()*scale;

                            part->initialQx = rotation.X();
                            part->initialQy = rotation.Y();
                            part->initialQz = rotation.Z();
                            part->initialQw = rotation.W();

                            gp_EulerSequence seq = gp_Extrinsic_XYZ;

                            Standard_Real roll;
                            Standard_Real pitch;
                            Standard_Real yaw;

                            rotation.GetEulerAngles(seq, roll, pitch, yaw);

                            part->initialRoll = roll;
                            part->initialPitch = pitch;
                            part->initialYaw = yaw;
                            
                            BRepMesh_IncrementalMesh mesh(localShape, model.resolution);

                            std::string stlPath ="meshes/" + partName + ".stl";

                            writer.Write(localShape, stlPath.c_str());
                            
                            part->stlPath = stlPath;
                        }
                        
                    }
                }
            }
        }
    }

    return true;
}