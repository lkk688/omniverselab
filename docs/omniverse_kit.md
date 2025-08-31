# Omniverse Kit

Build custom applications, extensions, and tools with NVIDIA Omniverse Kit - the powerful development platform for creating USD-based applications.

## Course: An Introduction to Developing With NVIDIA Omniverse

### Getting Started with Kit App Templates

**Prerequisites:**
- Access to the Kit App Template repository: https://github.com/NVIDIA-Omniverse/kit-app-template
- Development environment set up with appropriate tooling

#### Creating Your First Application

**Step 1: Initialize a New Template**

Open your terminal and run the template creation command for your operating system:

**Windows:**
```bash
.\repo.bat template new
```

**Linux/macOS:**
```bash
./repo.sh template new
```

This command creates a new application from Kit SDK resources using an interactive setup process.

**Step 2: Configure Your Application**

You'll be prompted with a series of configuration questions. Use the arrow keys to navigate options and press Enter to confirm each selection:

```
Do you accept the EULA (End User License Agreement)?
(Select Yes or No with arrow keys): Yes
[ctrl+c to Exit]
? Select with arrow keys what you want to create: Application>
? Select with arrow keys your desired template: [kit_base_editor]: Kit Base Editor
? Enter name of application .kit file [name-spaced, lowercase, alphanumeric]:
    my_company.my_editor
? Enter application_display_name: My Editor
? Enter version: 0.1.0
```

**Important:** Choose **Kit Base Editor** as your template for this tutorial.

#### Understanding the .kit File

**Location and Purpose:**
The generated `.kit` file can be found in `source\apps\` (or `source/apps/` on Linux/macOS). If you used the default name, it will be `my_company.my_editor.kit`.

**What is a .kit File?**
The `.kit` file serves as a **manifest** that defines:
- Which extensions to load from the Omniverse ecosystem
- Application configuration and dependencies
- Building blocks that compose your custom application

Each entry in the `.kit` file represents a modular component that contributes specific functionality to your application.

#### Building and Launching Your Application

**Step 1: Build the Application**

Before running your application, you must compile it and prepare its extensions:

**Windows:**
```bash
.\repo.bat build
```

**Linux/macOS:**
```bash
./repo.sh build
```

The build process compiles your application and its extensions, preparing them for launch.

**Step 2: Launch the Application**

Start your newly created application:

**Windows:**
```bash
.\repo.bat launch
```

**Linux/macOS:**
```bash
./repo.sh launch
```

When prompted in the terminal, press **Enter** to select your application and begin the launch process.

**Note:** The first launch may take several minutes as resources are initialized. Subsequent launches will be faster.

#### Exploring Your Application

**Default Interface:**
Your Kit Base Editor application includes four main menus:
- **File** - File operations and project management
- **Edit** - Editing tools and preferences
- **Create** - Content creation tools
- **Window** - Interface and panel management

**Menu Extensions in .kit File:**
These menus are defined by extensions in your `.kit` file:
```
"omni.kit.menu.create" = {} # Create menu
"omni.kit.menu.edit" = {}  # Edit menu
"omni.kit.menu.file" = {}  # File menu
```

#### Viewport Navigation

**Interactive Controls:**
- **Camera Movement:** Right-click and hold (RMB) + WASD keys
- **Camera Speed:** Middle scroll wheel while holding RMB
- **Camera Rotation:** Drag with RMB
- **Focus Object:** Press **F** key to center view on selected object
- **Combined Navigation:** Use RMB + drag + WASD simultaneously for fluid movement

### Creating a USD Explorer Application

Now let's create a more advanced application using the USD Explorer template, which provides enhanced tools for reviewing and working with large USD content.

#### USD Explorer Setup

**Step 1: Create USD Explorer Template**

Run the template command again:

**Windows:**
```bash
.\repo.bat template new
```

**Linux/macOS:**
```bash
./repo.sh template new
```

This time, select **USD Explorer** as your template. You'll see additional questions about "setup extension" configuration.

**Step 2: Compare Applications**

In VS Code:
1. Open both `.kit` files: `my_company.my_editor.kit` and `my_company.my_usd_explorer.kit`
2. Arrange them side by side for comparison
3. Examine the `[dependencies]` sections - note the different extensions each application includes

**Step 3: Build and Launch USD Explorer**

Build the USD Explorer application:

**Windows:**
```bash
.\repo.bat build
```

**Linux/macOS:**
```bash
./repo.sh build
```

Then launch it:

**Windows:**
```bash
.\repo.bat launch
```

**Linux/macOS:**
```bash
./repo.sh launch
```

When prompted, select your new USD Explorer application.

### Extensions: Building Blocks of Kit Applications

#### Understanding Extensions

**What is an Extension?**

Extensions are **isolated units of application functionality** that serve as the fundamental building blocks of Omniverse Kit-based applications. They can be developed using either:
- **Python** - For rapid development and scripting
- **C++** - For performance-critical functionality

**How Extensions Work:**

When an application starts up, it:
1. **Reads the .kit file** - References the list of required extensions
2. **Loads Extensions** - Initializes each extension as part of the application startup
3. **Integrates Functionality** - Extensions contribute to the application's appearance and capabilities

Each template includes several baseline extensions in its `.kit` file, with the expectation that developers will add additional extensions to build custom functionality.

#### Working with Extensions in Development Mode

**Enabling Developer Mode:**

To access extension management tools, launch your application with the `-d` parameter:

**USD Explorer with Developer Mode:**

**Windows:**
```bash
.\repo.bat launch -d
```

**Linux/macOS:**
```bash
./repo.sh launch -d
```

**Developer Mode Benefits:**
- **Extensions Window** - Access to extension browser and management
- **Developer Menu** - Additional development tools and options
- **Runtime Extension Management** - Enable/disable extensions without rebuilding

#### Extension Management Workflow

**Method 1: Runtime Extension Management (USD Explorer)**

1. **Launch with Developer Mode:** Use the `-d` parameter when launching
2. **Access Extensions Window:** Navigate to **Window > Extensions**
3. **Search and Install:** Use the search bar to find extensions (e.g., "measure")
4. **Enable Extensions:** Toggle extensions on/off as needed

**Example: Adding the Measure Tool**
1. Open the Extensions window
2. Search for "measure" in the search bar
3. Install and enable the Measure Tool extension
4. Access new functionality through the application interface

**Method 2: .kit File Configuration (Kit Base Editor)**

For Kit Base Editor applications, extensions are typically managed through direct `.kit` file modification:

1. **Launch with Developer Mode:**
   
   **Windows:**
   ```bash
   .\repo.bat launch -d
   ```
   
   **Linux/macOS:**
   ```bash
   ./repo.sh launch -d
   ```

2. **Interface Differences:**
   - **No Tools Menu** - Kit Base Editor has a simpler interface
   - **Developer Menu Available** - Accessible due to `-d` parameter
   - **Extension Management** - Primarily through `.kit` file editing

#### Extension Development Best Practices

**Planning Your Extensions:**
- **Modular Design** - Keep extensions focused on specific functionality
- **Dependency Management** - Clearly define extension dependencies
- **User Interface** - Design intuitive interfaces for extension features
- **Performance** - Consider the impact on application startup and runtime

**Integration Strategies:**
- **Template Selection** - Choose the appropriate base template for your needs
- **Extension Layering** - Build complex functionality through multiple cooperating extensions
- **Configuration Management** - Use `.kit` files effectively to manage extension combinations
