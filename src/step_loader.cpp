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

    IFSelect_ReturnStatus status =
        reader.ReadFile(filename);

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

    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());

    TDF_LabelSequence labels;

    shapeTool->GetFreeShapes(labels);

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
            
            StlAPI_Writer writer;std::filesystem::create_directories("meshes");

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

                            TopoDS_Shape shape = shapeTool->GetShape(component);
                            TopLoc_Location location = shape.Location();
                            gp_Trsf transform = location.Transformation();

                            gp_XYZ translation = transform.TranslationPart();
                            gp_Quaternion rotation = transform.GetRotation();

                            std::cout
                                << "Position: "
                                << translation.X() << " "
                                << translation.Y() << " "
                                << translation.Z()
                                << std::endl;
                            std::cout
                                << "Rotation: "
                                << rotation.X() << " "
                                << rotation.Y() << " "
                                << rotation.Z() << " "
                                << rotation.W()
                                << std::endl;
                            
                            BRepMesh_IncrementalMesh mesh(shape, model.resolution);

                            std::string stlPath ="meshes/" + partName + ".stl";

                            writer.Write(shape, stlPath.c_str());
                            
                            std::cout << stlPath << std::endl;
                        }
                        else{
                            std::cout << " : match found" << std::endl;
                        }
                        
                    }
                }
            }
        }
    }

    return true;
}