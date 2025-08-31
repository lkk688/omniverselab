#!/usr/bin/env python3
"""
Test script to demonstrate the enhanced GLB display functionality
with improved error handling and debugging features.
"""

import os
from myusddisplay import convert_usd_to_glb, display_glb_on_macos, display_glb_in_jupyter
from pxr import Usd, UsdGeom, Gf, Sdf

def first_stage(file_path = "output/first_stage.usda"):
    # Define a file path name:
    # Create a stage at the given `file_path`:
    # stage = Usd.Stage.CreateNew(file_path)#saved usda file
    # print(stage.ExportToString())
    print(f"Would create stage at: {file_path}")

def open_stage(file_path = "output/first_stage.usda"):
    # stage = Usd.Stage.Open(file_path)

    # Define a prim of type `Cube` at path `/hello`:
    # UsdGeom.Cube.Define(stage, "/hello")
    # Save the stage:
    # stage.Save()
    print(f"Would open and modify stage at: {file_path}")

    #DisplayUSD(file_path, show_usd_code=True)

def test_glb_displayonly():
    # Test browser display
    glb_file = "converted_glb/first_stage_37d560.glb"
    print("\n🌐 Testing browser display...")
    display_glb_on_macos(glb_file, method="browser")

def test_glb_display(usdz_file = "openusd/assets/box/cubebox_a02_distilled.usdz"):
    """Test GLB display with various scenarios"""
    
    print("🧪 Testing GLB Display Functionality")
    print("=" * 50)
    
    # Test with the working USDZ file
    #usdz_file = "openusd/assets/box/cubebox_a02_distilled.usdz"
    
    if os.path.exists(usdz_file):
        print(f"✅ Found test file: {usdz_file}")
        
        # Convert to GLB
        print("\n🔄 Converting USD to GLB...")
        glb_file = convert_usd_to_glb(usdz_file)
        print(f"✅ Converted to: {glb_file}")
        
        # Test browser display
        print("\n🌐 Testing browser display...")
        display_glb_on_macos(glb_file, method="browser")
        
        # Test Jupyter display (will show HTML object)
        print("\n📓 Testing Jupyter display...")
        jupyter_html = display_glb_in_jupyter(glb_file)
        print(f"✅ Jupyter HTML object created: {type(jupyter_html)}")
        
        # Test Quick Look display
        print("\n👁️ Testing Quick Look display...")
        display_glb_on_macos(glb_file, method="quicklook")
        
    else:
        print(f"❌ Test file not found: {usdz_file}")
        print("Please ensure the assets/box/cubebox_a02_distilled.usdz file exists.")

if __name__ == "__main__":
    #first_stage()#empty usda file
    #open_stage()#add cube
    #test_glb_display("assets/box/cubebox_a02_distilled.usdz")
    #test_glb_display("assets/first_stage.usda")  # USDA files work but contain minimal geometry
    
    from utils.visualization import CovertFile
    usd_filename = "openusd/assets/first_stage.usda"
    converted_file = CovertFile(usd_filename)
    print(f"✅ Successfully converted to: {converted_file}")
    print("\n🌐 Browser display...")
    display_glb_on_macos(converted_file, method="browser")
    #test_glb_display()
    #test_glb_displayonly()