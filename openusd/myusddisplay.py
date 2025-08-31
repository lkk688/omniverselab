#!/usr/bin/env python3
"""
USD to GLB Converter and macOS Viewer

This script converts USD/USDA files to GLB format and displays them using:
1. Web browser with Google's model-viewer (interactive, full-featured)
2. macOS Quick Look (native preview, basic viewing)
3. Jupyter Notebook inline display (embedded model-viewer)

Usage:
    # As a script
    python3 myusddisplay.py
    
    # As a module
    from myusddisplay import convert_usd_to_glb, display_glb_on_macos, display_glb_in_jupyter
    
    glb_file = convert_usd_to_glb("my_model.usda")
    display_glb_on_macos(glb_file, method="browser")  # or "quicklook"
    
    # For Jupyter notebooks
    from IPython.display import display
    html_viewer = display_glb_in_jupyter(glb_file)
    display(html_viewer)

Requirements:
    - usd2gltf command-line tool
    - macOS (for Quick Look method)
    - Internet connection (for model-viewer)
    - IPython/Jupyter (for notebook display)
"""

import os
import subprocess
from pathlib import Path as PathLib
from uuid import uuid4
from IPython.display import display, HTML
#pip install pythreejs
from pythreejs import *

def convert_usd_to_glb(usd_path: str, output_dir: str = "converted_glb") -> str:
    """
    Convert a USD file to glb format using `usd2gltf` and return the path to the .glb file.
    """
    usd_path = PathLib(usd_path).resolve()
    output_dir = PathLib(output_dir)
    output_dir.mkdir(exist_ok=True)
    
    output_file = output_dir / f"{usd_path.stem}_{uuid4().hex[:6]}.glb"
    
    cmd = [
        "usd2gltf",
        "--input", str(usd_path),
        "--output", str(output_file)
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode != 0:
        print("❌ Conversion failed:", result.stderr)
        raise RuntimeError("usd2gltf failed")
    else:
        print("✅ Converted to:", output_file)

    return str(output_file)

def display_glb_in_jupyter(glb_path: str, width: str = "100%", height: str = "600px"):
    """
    Display a GLB file directly in a Jupyter notebook using model-viewer.
    
    Args:
        glb_path: Path to the GLB file
        width: Width of the viewer (CSS format, e.g., "100%", "800px")
        height: Height of the viewer (CSS format, e.g., "600px", "400px")
    
    Returns:
        IPython.display.HTML object for rendering in Jupyter
    """
    import base64
    import os
    
    # Read GLB file and encode as base64
    try:
        with open(glb_path, 'rb') as f:
            glb_data = f.read()
        glb_base64 = base64.b64encode(glb_data).decode('utf-8')
        data_uri = f"data:model/gltf-binary;base64,{glb_base64}"
    except Exception as e:
        print(f"❌ Failed to read GLB file: {e}")
        return None
    
    # Create HTML content for Jupyter notebook
    html_content = f"""
<div style="text-align: center; padding: 10px;">
    <h3 style="color: #333; margin-bottom: 15px;">3D Model Viewer</h3>
    <script type="module" src="https://unpkg.com/@google/model-viewer/dist/model-viewer.min.js"></script>
    <model-viewer 
        id="jupyter-model-viewer-{uuid4().hex[:8]}"
        src="{data_uri}"
        alt="3D Model"
        style="width: {width}; height: {height}; background-color: #ffffff; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); --poster-color: transparent;"
        auto-rotate 
        camera-controls 
        loading="eager"
        reveal="auto"
        environment-image="neutral"
        shadow-intensity="1"
        exposure="1"
        tone-mapping="neutral">
        <div slot="progress-bar" style="color: #666; font-style: italic; padding: 20px;">
            Loading 3D model...
        </div>
    </model-viewer>
    <p style="margin-top: 15px; color: #666; font-size: 14px;">
        File: {os.path.basename(glb_path)} ({len(glb_data):,} bytes)<br>
        Use mouse to rotate, scroll to zoom
    </p>
    <div id="jupyter-debug-{uuid4().hex[:8]}" style="display: none; margin-top: 15px; padding: 10px; background-color: #e3f2fd; border-radius: 4px; font-size: 12px; color: #1976d2;">
        Debug information will appear here if there are issues.
    </div>
</div>
<script>
    console.log('Initializing Jupyter 3D Model Viewer...');
    
    // Wait for model-viewer to be defined
    customElements.whenDefined('model-viewer').then(() => {{
        console.log('model-viewer element is defined in Jupyter');
        
        const modelViewer = document.querySelector('#jupyter-model-viewer-{uuid4().hex[:8]}');
        const debugInfo = document.querySelector('#jupyter-debug-{uuid4().hex[:8]}');
        const loadingDiv = modelViewer.querySelector('[slot="progress-bar"]');
        
        // Add comprehensive error handling
        modelViewer.addEventListener('error', (event) => {{
            console.error('Jupyter model loading error:', event.detail);
            loadingDiv.textContent = 'Error loading 3D model';
            loadingDiv.style.color = '#d32f2f';
            loadingDiv.style.fontWeight = 'bold';
            
            if (debugInfo) {{
                debugInfo.style.display = 'block';
                debugInfo.innerHTML = `
                    <strong>Error Details:</strong><br>
                    Type: ${{event.detail?.type || 'Unknown'}}<br>
                    Message: ${{event.detail?.message || 'No message available'}}<br>
                    Data URI length: {len(glb_base64)} characters<br>
                    GLB file size: {len(glb_data):,} bytes
                `;
            }}
        }});
        
        modelViewer.addEventListener('load', () => {{
            console.log('Model loaded successfully in Jupyter notebook');
            if (loadingDiv) {{
                loadingDiv.style.display = 'none';
            }}
        }});
        
        modelViewer.addEventListener('progress', (event) => {{
            console.log('Jupyter loading progress:', event.detail);
        }});
        
        // Timeout fallback for Jupyter
        setTimeout(() => {{
            if (loadingDiv && loadingDiv.textContent === 'Loading 3D model...') {{
                console.warn('Jupyter model loading timeout');
                if (debugInfo) {{
                    debugInfo.style.display = 'block';
                    debugInfo.innerHTML = `
                        <strong>Loading Timeout:</strong><br>
                        The model is taking longer than expected to load.<br>
                        Data URI length: {len(glb_base64)} characters<br>
                        GLB file size: {len(glb_data):,} bytes<br>
                        This might be due to large file size or network issues.
                    `;
                }}
            }}
        }}, 15000); // 15 second timeout for Jupyter
    }}).catch((error) => {{
        console.error('Failed to initialize model-viewer in Jupyter:', error);
        const loadingDiv = document.querySelector('[slot="progress-bar"]');
        if (loadingDiv) {{
            loadingDiv.textContent = 'Failed to initialize 3D viewer';
            loadingDiv.style.color = '#d32f2f';
            loadingDiv.style.fontWeight = 'bold';
        }}
    }});
</script>
    """
    
    print(f"✅ Displaying GLB file in Jupyter: {glb_path}")
    print(f"📊 GLB file size: {len(glb_data):,} bytes")
    
    return HTML(html_content)

def display_glb_on_macos(glb_path: str, method: str = "browser"):
    """
    Display a GLB file on macOS using different methods.
    
    Args:
        glb_path: Path to the GLB file
        method: Display method - "browser" (web browser with model-viewer) or "quicklook" (macOS Quick Look)
    """
    import os
    
    if method == "quicklook":
        # Note: macOS Quick Look doesn't natively support GLB files
        # Try to open with default application instead
        try:
            # First try with the default application
            result = os.system(f'open "{os.path.abspath(glb_path)}"')
            if result == 0:
                print(f"✅ Opening GLB file with default application: {glb_path}")
                print("💡 GLB file opened with the default 3D model viewer (if available).")
            else:
                raise Exception("Default application failed to open GLB file")
        except Exception as e:
            print(f"❌ Quick Look/Default app doesn't support GLB files: {e}")
            print("ℹ️  GLB files require specialized 3D viewers or web browsers.")
            print("🔄 Falling back to browser method...")
            return display_glb_on_macos(glb_path, method="browser")
    
    elif method == "browser":
        import webbrowser
        import tempfile
        import base64
        
        # Read GLB file and encode as base64 to avoid CORS issues
        try:
            with open(glb_path, 'rb') as f:
                glb_data = f.read()
            glb_base64 = base64.b64encode(glb_data).decode('utf-8')
            data_uri = f"data:model/gltf-binary;base64,{glb_base64}"
        except Exception as e:
            print(f"❌ Failed to read GLB file: {e}")
            return None
        
        # Create a temporary HTML file with model-viewer using base64 data
        html_content = f"""
<!DOCTYPE html>
<html>
<head>
    <title>3D Model Viewer</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <script type="module" src="https://unpkg.com/@google/model-viewer/dist/model-viewer.min.js"></script>
    <style>
        body {{
            margin: 0;
            padding: 20px;
            font-family: Arial, sans-serif;
            background-color: #f5f5f5;
        }}
        .container {{
            max-width: 1200px;
            margin: 0 auto;
            text-align: center;
        }}
        h1 {{
            color: #333;
            margin-bottom: 20px;
        }}
        model-viewer {{
            width: 100%;
            height: 600px;
            background-color: #ffffff;
            border-radius: 8px;
            box-shadow: 0 4px 8px rgba(0,0,0,0.1);
            --poster-color: transparent;
        }}
        .loading {{
            color: #666;
            font-style: italic;
            padding: 20px;
        }}
        .error {{
            color: #d32f2f;
            font-weight: bold;
            padding: 20px;
        }}
        .debug-info {{
            margin-top: 20px;
            padding: 10px;
            background-color: #e3f2fd;
            border-radius: 4px;
            font-size: 12px;
            color: #1976d2;
        }}
    </style>
</head>
<body>
    <div class="container">
        <h1>3D Model Viewer</h1>
        <model-viewer 
            id="model-viewer"
            src="{data_uri}"
            alt="3D Model"
            auto-rotate 
            camera-controls 
            loading="eager"
            reveal="auto"
            environment-image="neutral"
            shadow-intensity="1"
            exposure="1"
            tone-mapping="neutral">
            <div slot="progress-bar">
                <div class="loading">Loading 3D model...</div>
            </div>
        </model-viewer>
        <p style="margin-top: 20px; color: #666;">
            File: {os.path.basename(glb_path)} ({len(glb_data):,} bytes)<br>
            Use mouse to rotate, scroll to zoom
        </p>
        <div id="debug-info" class="debug-info" style="display: none;">
            Debug information will appear here if there are issues.
        </div>
    </div>
    <script>
        console.log('Initializing 3D Model Viewer...');
        
        // Wait for model-viewer to be defined
        customElements.whenDefined('model-viewer').then(() => {{
            console.log('model-viewer element is defined');
            
            const modelViewer = document.querySelector('#model-viewer');
            const debugInfo = document.querySelector('#debug-info');
            const loadingDiv = document.querySelector('.loading');
            
            // Add comprehensive error handling
            modelViewer.addEventListener('error', (event) => {{
                console.error('Model loading error:', event.detail);
                loadingDiv.textContent = 'Error loading 3D model';
                loadingDiv.className = 'error';
                
                debugInfo.style.display = 'block';
                debugInfo.innerHTML = `
                    <strong>Error Details:</strong><br>
                    Type: ${{event.detail?.type || 'Unknown'}}<br>
                    Message: ${{event.detail?.message || 'No message available'}}<br>
                    Data URI length: {len(glb_base64)} characters<br>
                    GLB file size: {len(glb_data):,} bytes
                `;
            }});
            
            modelViewer.addEventListener('load', () => {{
                console.log('Model loaded successfully');
                loadingDiv.style.display = 'none';
            }});
            
            modelViewer.addEventListener('progress', (event) => {{
                console.log('Loading progress:', event.detail);
            }});
            
            // Check if model-viewer is ready
            modelViewer.addEventListener('model-visibility', (event) => {{
                console.log('Model visibility changed:', event.detail);
            }});
            
            // Timeout fallback
            setTimeout(() => {{
                if (loadingDiv.textContent === 'Loading 3D model...') {{
                    console.warn('Model loading timeout - still showing loading message');
                    debugInfo.style.display = 'block';
                    debugInfo.innerHTML = `
                        <strong>Loading Timeout:</strong><br>
                        The model is taking longer than expected to load.<br>
                        Data URI length: {len(glb_base64)} characters<br>
                        GLB file size: {len(glb_data):,} bytes<br>
                        Check browser console for more details.
                    `;
                }}
            }}, 10000); // 10 second timeout
        }}).catch((error) => {{
            console.error('Failed to initialize model-viewer:', error);
            document.querySelector('.loading').textContent = 'Failed to initialize 3D viewer';
            document.querySelector('.loading').className = 'error';
        }});
    </script>
</body>
</html>
        """
        
        # Create temporary HTML file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.html', delete=False) as f:
            f.write(html_content)
            temp_html = f.name
        
        try:
            # Open in default web browser
            webbrowser.open(f'file://{temp_html}')
            print(f"✅ Opening GLB file in browser: {glb_path}")
            print(f"📁 Temporary HTML file: {temp_html}")
            print(f"📊 GLB file size: {len(glb_data):,} bytes")
            print("💡 The HTML file will remain until you manually delete it.")
            return temp_html
        except Exception as e:
            print(f"❌ Failed to open in browser: {e}")
            print(f"📄 You can manually open: {temp_html}")
            return temp_html
    
    else:
        print(f"❌ Unknown method: {method}. Use 'browser' or 'quicklook'.")
        return None

def main():
    """Example usage of the USD to GLB conversion and display functions."""
    usd_file = "/Users/kaikailiu/Documents/MyRepo/omniverselab/openusd/assets/box/cubebox_a02_distilled.usdz"
    #usd_file = "openusd/assets/first_stage.usda"
    print("🔄 Converting USD to GLB...")
    glb_file = convert_usd_to_glb(usd_file)
    
    print("\n📋 Choose display method:")
    print("1. Browser (model-viewer) - Interactive web-based viewer")
    print("2. Quick Look - macOS native preview")
    print("3. Jupyter Notebook - Display inline in notebook")
    
    choice = input("Enter choice (1, 2, or 3, default=1): ").strip() or "1"
    
    if choice == "2":
        display_glb_on_macos(glb_file, method="quicklook")
    elif choice == "3":
        # For Jupyter notebook display
        html_obj = display_glb_in_jupyter(glb_file)
        if html_obj:
            display(html_obj)
    else:
        display_glb_on_macos(glb_file, method="browser")

if __name__ == "__main__":
    main()