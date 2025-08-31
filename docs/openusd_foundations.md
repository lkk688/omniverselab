# OpenUSD Foundations

Developed by Pixar Animation Studios, OpenUSD is an open-source framework for creating, simulating, and collaborating in 3D worlds. OpenUSD serves as the foundational technology for NVIDIA Omniverse.

The universal structure of OpenUSD allows developers to integrate multiple assets into a single "stage," enabling dynamic scene manipulation. Each OpenUSD asset contains independent data layer stacks, such as geometry, shading, or textures, which can be interchanged without affecting other elements. For example, in an OpenUSD stage representing a kitchen environment, the data layers for a chair or stove can be swapped independently, enabling rapid scene updates.

## Important Resources

- [OpenUSD Documentation](https://docs.omniverse.nvidia.com/usd/latest/index.html)
- [OpenUSD GitHub Repository](https://github.com/PixarAnimationStudios/OpenUSD)
- [OpenUSD Learning Path](https://www.nvidia.com/en-us/learn/learning-path/openusd/)

## Learning Path Overview

Learn OpenUSD, the core of Omniverse's scene representation format. The [NVIDIA OpenUSD Learning Path](https://www.nvidia.com/en-us/learn/learning-path/openusd/) contains 7 comprehensive courses covering OpenUSD fundamentals and advanced topics.

Install `usd-core`
```bash
pip install usd-core
pip install usd2gltf #USD to glTF conversion using usd2gltf
which usd2gltf
```
Test creation
```bash
python -c "from pxr import Usd, UsdGeom; stage = Usd.Stage.CreateInMemory(); cube = UsdGeom.Cube.Define(stage, '/hello'); stage.Export('test_cube.usda'); print('Test USD file created successfully')"
```
usd2gltf conversion
```bash
(mypy311) kaikailiu@Kaikais-MacBook-Pro omniverselab % usd2gltf --input test_cube.usda --output test_cube.glb && echo 'USD to glTF conversion successful'
Converting: test_cube.usda
To: test_cube.glb
Converted!
USD to glTF conversion successful
```


## Course 1: Learning About Stages, Prims, and Attributes

**[Course Link](https://learn.nvidia.com/courses/course-detail?course_id=course-v1:DLI+S-OV-17+V1)**

This course covers the fundamental concepts of OpenUSD:

- **Create and manipulate USD files** - Learn to set up USD files from scratch, establishing the foundation for 3D scenes
- **Define primitives** - Gain hands-on experience with defining various types of prims, the building blocks of USD
- **Establish scene hierarchies** - Organize and structure 3D elements effectively for coherent and manageable scenes
- **Add dynamic lighting** - Implement lighting systems to bring scenes to life with enhanced visual appeal
- **Manage attributes and metadata** - Master the details of setting, getting, and manipulating essential scene elements
- **Traverse and inspect USD files** - Develop skills to navigate through USD files and understand scene details
- **Verify prim existence** - Learn techniques to check for specific prims, ensuring scene integrity and completeness

### Stage

An **OpenUSD stage** represents the scenegraph, which defines the contents and structure of a scene. It consists of a hierarchy of objects called prims, which can represent geometry, materials, lights, and other organizational elements. The scene is stored as a data structure of connected nodes, hence the term "scenegraph."

A stage can be composed of:
- A single USD file (e.g., a robot asset)
- Multiple USD files combined together (e.g., a factory containing many robot assets)

The stage represents the composed result of all contributing files or layers. **Composition** is the algorithm that determines how USD files (or layers) should be assembled and combined into the final scene.

**Benefits of OpenUSD stages:**

- **Modularity** - Enable modification of individual elements without altering original files ("non-destructive" editing)
- **Scalability** - Efficiently manage large datasets through features like payloads and composition

```python
# Create a new, empty USD stage where 3D scenes are assembled
Usd.Stage.CreateNew()
  
# Open an existing USD file as a stage
Usd.Stage.Open()
  
# Saves all layers in a USD stage
Usd.Stage.Save()
```

### Hydra Rendering Architecture

**Hydra** is a powerful rendering architecture within OpenUSD that enables efficient and flexible rendering of complex 3D scenes. It serves as a bridge between scene description data (USD) and rendering backends (OpenGL, DirectX, Metal, Vulkan).

**Key Features:**

- **Extensible Architecture** - Supports multiple renderers including Arnold, RenderMan, and others
- **Built-in Renderers** - Ships with HdStorm (real-time renderer), HdTiny, and HdEmbree
- **Render Delegates** - Plugin system for custom rendering backends
- **Wide Adoption** - Used by usdview and many other USD-based tools

**HdStorm Renderer:**
The included real-time OpenGL/Metal/Vulkan render delegate that provides out-of-the-box visualization capabilities for developers.

**Programming Interface:**
Interaction with Hydra is typically done through the **HydraPython** API, which provides programmatic control over the rendering process.

### USD File Formats

OpenUSD supports four core file formats for storing and exchanging 3D scene data, including meshes, cameras, lights, and shaders. All formats can be accessed through Python bindings.

#### USDA (.usda) - ASCII Format
- **Human-readable** ASCII text files encoding scene descriptions
- **Editable** - Can be manually edited and inspected
- **Use case** - Optimal for small files and stages referencing external content
- **Benefits** - Easy debugging and version control

#### USDC (.usdc) - Binary Crate Format
- **Compressed binary** format for efficient storage
- **Performance optimized** - Minimizes load times through compression and memory mapping
- **Use case** - Ideal for numerically-heavy data like geometry
- **Benefits** - Faster file access and reduced file sizes

#### USD (.usd) - Flexible Format
- **Format agnostic** - Can be either ASCII or binary
- **Interchangeable** - Format can be changed without breaking references
- **Debugging friendly** - Binary assets can be converted to ASCII for inspection

#### USDZ (.usdz) - Archive Format
- **Packaged delivery** - Uncompressed ZIP archive containing all necessary assets
- **Distribution ready** - Ideal for shipping complete, finalized assets
- **Limitation** - Not suitable for assets still under development

### Prim (Primitives)

Primitives, or **prims**, are the fundamental building blocks of any OpenUSD scene. Understanding prims is essential for anyone working with 3D content creation and manipulation in the OpenUSD ecosystem.

**What is a Prim?**
A prim is a container that holds various types of data, attributes, and relationships which define an object or entity within a scene. Prims can represent:
- **Imageable entities** - Meshes, lights, cameras
- **Non-imageable entities** - Materials, transforms (xforms), organizational elements

**Hierarchical Structure:**
Prims are organized in a hierarchical structure, creating a scenegraph that represents relationships and transformations between objects.

**Prim Paths:**
Each prim has a unique identifier called a **path** for location within the scene graph.

*Example:* `/World/BuildingA/Geometry/building_geo`
- `building_geo` is a child of `Geometry`
- `Geometry` is a child of `BuildingA`
- `BuildingA` is a child of the root `World`

**Python API for Prims:**
```python
# Generic USD API command. Used to define a new prim on a stage at a specified path, and optionally the type of prim.
stage.DefinePrim(path, prim_type)

# Specific to UsdGeom schema. Used to define a new prim on a USD stage at a specified path of type Xform. 
UsdGeom.Xform.Define(stage, path)
	
# Retrieves the children of a prim. Useful for navigating through the scenegraph.
prim.GetChildren()
	
# Returns the type of the prim, helping identify what kind of data the prim contains.
prim.GetTypeName()

# Returns all properties of the prim.
prim.GetProperties()
```

### Scope

In OpenUSD, a **scope** is a special type of prim used primarily as a grouping mechanism in the scenegraph. It serves as an organizational container without representing any geometry or renderable content.

**Key Characteristics:**
- **Organizational tool** - Acts like a folder for organizing related prims
- **Non-transformable** - Cannot be transformed, promoting lightweight usage
- **Logical grouping** - Useful for grouping materials, animation, or geometry prims

**Use Cases:**
- Grouping all material-related prims
- Organizing animation elements
- Structuring geometry hierarchies

**Python API for Scope:**
```python
# Used to define a new scope at a specified path on a given stage
UsdGeom.Scope.Define(stage, path)

# This command is generic, but it's useful to confirm that a prim's type is a scope, ensuring correct usage in scripts
prim.IsA(UsdGeom.Scope)
```

### Xform (Transform)

In OpenUSD, an **xform** is a type of prim that stores transformation data (translation, rotation, scaling) which applies to its child prims. Xforms are powerful tools for grouping and manipulating the spatial arrangement of objects in 3D scenes.

**Purpose:**
- **Spatial transformation** - Defines the coordinate space for child prims
- **Hierarchical transforms** - Transformations cascade down to children
- **Scene organization** - Groups related objects under common transforms

**Transform Operations:**
Xforms support multiple transformation operations that can be combined in specific orders. The order of operations is crucial as different sequences yield different results.

**Python API for Xform:**
```python
# Used to define a new Xform prim at a specified path on a given stage
UsdGeom.Xform.Define(stage, path)

# Retrieves the order of transformation operations, which is crucial for understanding how multiple transformations are combined. Different orders can yield different results, so understanding XformOpOrder is important. 
xform.GetXformOpOrderAttr()
	
# Adds a new transform operation to the xform prim, such as translation or rotation, with specified value   
xform.AddXformOp(opType, value)
```

### USD Modules

The USD code repository consists of four core packages: **base**, **usd**, **imaging**, and **usdImaging**. For basic USD authoring and reading, you only need the **base** and **usd** packages.

**Core Modules:**

#### Usd Module
- **Purpose** - Core client-facing module for authoring, composing, and reading USD
- **Functionality** - Interface for creating/opening stages and interacting with prims, properties, metadata, and composition arcs

#### Sdf Module (Scene Description Foundation)
- **Purpose** - Foundations for serializing scene description to text-based file format
- **Functionality** - Implements scene description layers (SdfLayer) and manages prim/property paths

#### Gf Module (Graphics Foundation)
- **Purpose** - Foundation classes and functions for graphics operations
- **Functionality** - Linear algebra, mathematical operations, basic geometry, and 3D data types

**Schema Modules:**
Schemas are grouped into domains, each with its own module:
- **UsdGeom** - Geometry data
- **UsdShade** - Materials and shaders
- **UsdLux** - Lighting
- **UsdPhysics** - Physics scene description

**Python Import:**
```python
# Import Usd, Sdf, and Gf libraries from Pixar
from pxr import Usd, Sdf, Gf
```

**UsdLux Example:**
```python
# Import the UsdLux module
from pxr import UsdLux
	
# Create a sphere light primitive
UsdLux.SphereLight.Define(stage, '/path/to/light')

# Set the intensity of a light primitive
light_prim.GetIntensityAttr().Set(500)
```

### USD Properties: Attributes

USD **attributes** are one of two types of USD properties (the other being relationships). Properties describe the characteristics of prims within a USD scene.

**What are Attributes?**
- **Data storage** - Store values that define appearance, behavior, or other prim properties
- **Schema-specific** - Each schema provides specific APIs for accessing its attributes
- **Typed values** - Support various data types (floats, vectors, colors, etc.)

**Working with Attributes:**
Generally use schema-specific APIs rather than generic property access.

```python
# Get the radius value of sphere_prim that is of type UsdGeom.Sphere
sphere_prim.GetRadiusAttr().Get()

# Set the double-sided property of the prim
sphere_prim.GetDoubleSidedAttr().Set(True)
```

### USD Properties: Relationships

**Relationships** establish connections between prims, acting as pointers or links between objects in the scene hierarchy. They enable prims to target or reference other prims, attributes, or relationships, establishing dependencies between scenegraph objects.

**Key Functions:**
- **Connect prims** - Link objects in the scene hierarchy
- **Reference data** - Point to attributes or other relationships
- **Establish dependencies** - Create data flow between scene elements

```python
# Get the target paths of a relationship
UsdRelationship.GetTargets()

# Set the target paths for a relationship
UsdRelationship.SetTargets()

# Add a new target path to a relationship
UsdRelationship.AddTarget()

# Remove a target path from a relationship
UsdRelationship.RemoveTarget()
```

### Primvars (Primitive Variables)

**Primvars** are special attributes that enable efficient management and manipulation of hierarchical object data in complex 3D scenes. They address key computer graphics challenges:

**Problems Solved:**
- **Shader binding** - Bind user data on geometric primitives for shader access during rendering
- **Surface interpolation** - Specify values associated with vertices/faces that interpolate across primitive surfaces
- **Data inheritance** - Inherit attributes down namespace hierarchies for sparse authoring

**Python API for Primvars:**
```python
# Constructs a UsdGeomPrimvarsAPI on UsdPrim prim
primvar_api = UsdGeom.PrimvarsAPI(prim)

# Creates a new primvar called displayColor of type Color3f[]
primvar_api.CreatePrimvar('displayColor', Sdf.ValueTypeNames.Color3fArray)

# Gets the displayColor primvar
primvar = primvar_api.GetPrimvar('displayColor')

# Sets displayColor values
primvar.Set([Gf.Vec3f(0.0, 1.0, 0.0)])

# Gets displayColor values
values = primvar.Get()
```

### XformCommonAPI

**XformCommonAPI** is a component of the OpenUSD framework that facilitates authoring and retrieval of common transformation operations. This API provides a single interface for translation, rotation, scale, and pivot operations that maintains compatibility with import/export workflows across various tools.

**Purpose:**
- **Simplified transforms** - Common transformation interface
- **Tool compatibility** - Compatible with import/export across many tools
- **Standardized interchange** - Simplifies transformation data exchange

### Stage Traversal

**Stage traversal** enables efficient navigation and manipulation of the scenegraph. You can iterate through child prims, access parent prims, and traverse hierarchies to find specific prims of interest.

**Key Concepts:**
- **Navigation** - Move through the scene hierarchy programmatically
- **Filtering** - Use predicates to filter traversal results
- **Efficiency** - Optimized algorithms for large scene graphs

**Traversal Methods:**
Traversal works via the `Usd.PrimRange` class, with `stage.Traverse()` as a common convenience method.

```python
# Open a USD file and create a Stage object
stage = Usd.Stage.Open('car.usda')

# Traverses the stage of prims that are active
stage.Traverse()

# Define a predicate to filter prims that are active and loaded
predicate = Usd.PrimIsActive & Usd.PrimIsLoaded

# Traverse starting from the given prim and based on the predicate for filtering the traversal
Usd.PrimRange(prim, predicate=predicate)
```

## Course 2: Working With Prims and Default Schemas

**[Course Link](https://learn.nvidia.com/courses/course?course_id=course-v1:DLI+S-OV-20+V1&unit=block-v1:DLI+S-OV-20+V1+type@vertical+block@7420568df58a46afb3b4673a0f637f3f)**

### Specifiers

**Specifiers** in OpenUSD convey the intent for how a prim or primSpec should be interpreted in the composed scene. There are three types of specifiers:

- **Def** (Define) - Defines the prim in the current layer, indicating the prim exists and is available for processing
- **Over** - Provides overrides or additional data for an existing prim
- **Class** - Defines a class that can be inherited by other prims

```python
# Get a prim’s specifier
prim.GetSpecifier()

# Set a prim’s specifier
prim.SetSpecifier(specifier)

```

Every prim has a specifier. To make a prim present on the stage and available for processing, you define (`def`) that prim. You can use override specifiers (`over`) to hold opinions that will be applied to prims in another layer, enabling non-destructive editing workflows. Class specifiers (`class`) can be used to set up a collection of opinions and properties to be composed by other prims.

### Prim Paths

In OpenUSD, a **path** represents the location of a prim within a scene hierarchy. The string representation consists of prim names separated by forward slashes (`/`), similar to file system paths.

**Path Structure:**
- **Root** - Represented by a forward slash (`/`)
- **Hierarchy** - Each level separated by `/`
- **Example** - `/World/Geometry/Box` represents:
  - `Box` is a child of `Geometry`
  - `Geometry` is a child of `World`
  - `World` is a child of the root

```python
# Import the Sdf class
from pxr import Sdf

# Return the path of a Usd.Prim as an Sdf.Path object
Usd.Prim.GetPath()

# Retrieve a Usd.Prim at the specified path from the Stage
Usd.Stage.GetPrimAtPath()
```

### Default Prim

A **default prim** is a top-level prim that serves as the primary entry point for a stage. It's part of the scene's metadata and acts as the "control point" that helps tools and applications know where to start.

**Importance:**
- **Best practice** - Should be set on all stages
- **Tool compatibility** - Required by many tools and applications
- **Validation** - `usdchecker` reports errors if not set
- **Referencing** - Eliminates need to specify target prim when referencing stages

```python
from pxr import Usd, UsdGeom, Sdf

# Create a new USD stage
stage = Usd.Stage.CreateInMemory()

# Define a top-level Xform prim
default_prim = UsdGeom.Xform.Define(stage, Sdf.Path("/World")).GetPrim()

# Set the Xform prim as the default prim
stage.SetDefaultPrim(default_prim)

# Export the stage to a string to verify
usda = stage.GetRootLayer().ExportToString()
print(usda)

# Check that the expected default prim was set
assert stage.GetDefaultPrim() == default_prim
```

### Schemas

**Schemas** give meaning to prims in OpenUSD by defining "What is this element? What capabilities does it have?" They define data models and APIs for encoding and interchanging 3D and non-3D concepts through OpenUSD.

**Purpose:**
- **Data models** - Define structure and behavior of scene elements
- **Blueprints** - Provide templates for authoring and retrieving data
- **Interoperability** - Ensure consistent data interpretation across tools
- **Extensibility** - Allow custom schemas for specialized use cases

**Schema Types:**
- **Typed schemas** - Define specific prim types (e.g., UsdGeom.Mesh)
- **API schemas** - Add functionality to existing prims (e.g., UsdGeom.PrimvarsAPI)

```python
# Retrieve the schema info for a registered schema
Usd.SchemaRegistry.FindSchemaInfo()

# Retrieve the schema typeName
Usd.SchemaRegistry.GetSchemaTypeName()

```

#### IsA Schemas (Typed Schemas)

There are two main types of schemas in OpenUSD: **IsA schemas** and **API schemas**.

**IsA schemas**, also known as **Typed schemas** or **Prim schemas**, define what a prim fundamentally is. Each prim can only have one IsA schema at a time, assigned through the `typeName` metadata.

IsA schemas are derived from the core class `UsdTyped`, the base class for all typed schemas. They can be:

- **Concrete schemas** - Can be instantiated as prims in USD scenes (e.g., `UsdGeomMesh`, `UsdGeomScope`). They provide both a name and a typeName in their definition.
- **Abstract schemas** - Serve as base classes for related concrete schemas (e.g., `UsdGeomPointBased` serves as a base for geometric objects containing points like meshes and curves). They provide a name but no typeName.

**Common Schema Libraries:**

**UsdGeom** defines schemas for representing geometric objects, such as meshes, cameras, and curves. It also includes schemas for transformations, visibility, and other common properties.

```python
# Import related classes
from pxr import UsdGeom

# Define a sphere in the stage
sphere = UsdGeom.Sphere.Define(stage, "/World/Sphere")

# Get and Set the radius attribute of the sphere
sphere.GetRadiusAttr().Set(10)
```

**UsdLux** defines schemas for representing light sources in a scene, including sphere lights, disk lights, and distant lights.

Examples include `UsdLuxDiskLight`, `UsdLuxRectLight`, and `UsdLuxSphereLight`.

```python
# Import related classes
from pxr import UsdLux

# Define a disk light in the stage
disk_light = UsdLux.DiskLight.Define(stage, "/World/Lights/DiskLight")

# Get all Attribute names that are a part of the DiskLight schema
dl_attribute_names = disk_light.GetSchemaAttributeNames()

# Get and Set the intensity attribute of the disk light prim
disk_light.GetIntensityAttr().Set(1000)
```

#### API Schemas

**API schemas** are similar to IsA schemas but do not specify a `typeName`, making them non-concrete. They add functionality to existing prims without changing their fundamental type.

**Key Characteristics:**
- Named with the suffix "API" (e.g., `UsdShadeConnectableAPI`)
- Properties are namespaced with the schema's base name (e.g., `UsdPhysics.RigidBodyAPI.CreateVelocityAttr()` creates `physics:velocity`)
- Can be **single-apply** (applied once per prim) or **multiple-apply** (applied multiple times with different instance names)
- Listed in the `apiSchemas` metadata and queryable via the `HasAPI()` method
- Applied to already-typed prims to add specialized behaviors

**Example: UsdPhysicsRigidBodyAPI** adds physics properties to any `UsdGeomXformable` object for rigid body dynamics simulation.
```python
# Import related classes
from pxr import UsdPhysics

# Apply a UsdPhysics Rigidbody API on the cube prim
cube_rb_api = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())

# Get the Kinematic Enabled Attribute 
cube_rb_api.GetKinematicEnabledAttr()

# Create a linear velocity attribute of value 5
cube_rb_api.CreateVelocityAttr(5)
```

## Course 3: Learn OpenUSD: Using Attributes 

**[Course Link](https://learn.nvidia.com/courses/course-detail?course_id=course-v1:DLI+S-OV-19+V1)**
### Retrieving Attributes

```python
# Get the property names of the cube prim
cube_prop_names = cube.GetPrim().GetPropertyNames()
```

### Value Resolution

**Value resolution** is the algorithm by which final values for properties or metadata are compiled from all sources. The algorithm processes an ordered list of values including:

- **Default values** - Schema-defined defaults
- **Time samples** - Animated values at specific time codes
- **Fallback values** - Values from composition layers

The algorithm returns the resolved value based on composition strength ordering and time sampling rules.

Value resolution allows OpenUSD to provide a rich set of composition semantics while keeping the core lightweight and performant for random access to composed data.

### Metadata

**Metadata** in OpenUSD refers to name-value pairs that provide additional, non-animatable information attached to prims or their properties. It allows you to add custom information or annotations to scene description elements without modifying the underlying schema or data model.

Metadata is stored separately from the primary data and can be accessed and modified independently. It's typically used to store supplementary information not directly related to the geometry or rendering of an object.

**Key Differences Between Metadata and Attributes:**
- **Schema integration** - Metadata is separate from the core schema, while attributes are part of the schema definition
- **Purpose** - Metadata stores supplementary information, while attributes store data directly related to object properties or behavior
- **Time sampling** - Metadata cannot be sampled over time (no timesamples), making it more efficient to evaluate and store than attribute values

```python
# Retrieve the metadata value associated with the given key for a USD Object
usdobject.GetMetadata('key')

# Set the metadata value for the given key on a USD Object
usdobject.SetMetadata('key', value)

# Retrieve the metadata value associated with the given key for the stage
stage.GetMetadata('key')

# Set the metadata value for the given key on the stage
stage.SetMetadata('key', value)

# Use for better performance if accessing a single value and not all the metadata within a key
GetMetadataByDictKey()
```

### Custom Attributes

**Custom attributes** in OpenUSD are user-defined properties that can be added to prims to store additional data. Unlike schema attributes, which are predefined and standardized, custom attributes allow users to extend OpenUSD's functionality at runtime to suit specific requirements.

**Common Use Cases:**
- **Metadata storage** - Additional information about a prim (author names, creation dates, custom tags)
- **Animation data** - Custom animation curves or parameters not covered by standard schema attributes
- **Simulation parameters** - Parameters for physics simulations or procedural generation processes
- **Arbitrary end user data** - Runtime-defined custom data for specialized workflows

```python
stage = Usd.Stage.CreateInMemory()
prim = stage.DefinePrim("/ExamplePrim", "Xform")
serial_num_attr = prim.CreateAttribute("serial_number", Sdf.ValueTypeNames.String)

assert serial_num_attr.IsCustom()

mtce_date_attr = prim.CreateAttribute("maintenance_date", Sdf.ValueTypeNames.String)
serial_num_attr.Set("qt6hfg23")
mtce_date_attr.Set("20241004")

print(f"Serial Number: {serial_num_attr.Get()}")
print(f"Last Maintenance Date: {mtce_date_attr.Get()}")
```
## Course 4: Learn OpenUSD: Traversing Stages

Stage traversal is the process of navigating through a stage's scenegraph to query or edit scene data. You can traverse the scenegraph by iterating through child prims, accessing parent prims, and navigating the hierarchy to find specific prims of interest.

### Traversal Methods

Stage traversal operates through the `Usd.PrimRange` class. Other methods like `stage.Traverse()` use `Usd.PrimRange` as their foundation.

```python
# Open a USD file and create a Stage object
stage = Usd.Stage.Open('car.usda') 

# Traverses the stage of prims that are active
stage.Traverse() 

# Define a predicate to filter prims that are active and loaded
predicate = Usd.PrimIsActive & Usd.PrimIsLoaded

# Traverse starting from the given prim and based on the predicate for filtering the traversal
Usd.PrimRange(prim, predicate=predicate)

```

### Active vs. Inactive Prims

In OpenUSD, all prims are **active** by default. Making a prim inactive models a non-destructive deletion from a stage. Deactivating a prim provides a way to temporarily remove (or prune) prims and their descendants from composition and processing, which can make traversals more efficient.

Active prims and their active children are visited and processed during stage traversals and operations. However, making a prim inactive by setting its "active" metadata to `false` prevents that prim from being visited and prevents its descendant prims from being composed onto the stage.

```python
# Make the prim at /Parent inactive
stage.GetPrimAtPath('/Parent').SetActive(False)

```

### Python API for Active State

Use these functions to manage prim active state:

- `UsdPrim.SetActive(bool)` - Set the "active" metadata for a prim
- `UsdPrim.IsActive()` - Return whether a prim is currently active on the stage

## Course 5: Learn OpenUSD: Understanding Model Kinds

Model kinds are metadata that organize and categorize different types of scene elements or prims into a hierarchical structure. Understanding kinds enables the creation of modular, reusable assets and can significantly impact how well you manage complex 3D scenes.

### Kind Categories

Kinds are predefined categories that define the role and behavior of different prims within the scene hierarchy. The main kinds include:

- **Component** - Self-contained, referenceable assets
- **Group** - Organizational units for related models
- **Assembly** - Containers for combining parts into larger entities

The base class for group and component kinds is **Model**, which should not be assigned as any prim's kind.

#### Component
A **component** is a reusable, self-contained asset that is complete and referenceable. Think of component models as consumer-facing products like a pen or a house. While drastically different in scale, both would be logical component models in a hierarchy.

#### Subcomponent
A component cannot contain other component models as descendants, which is why we have **subcomponents**. Subcomponents aren't model kinds in the traditional sense, but they identify important prims within a component.

#### Groups and Assemblies
All parents of a component model must have their kind metadata set to **group** or **assembly**:

- **Group** - An organizational unit used to logically group related models together
- **Assembly** - A subkind of group that serves as a container for combining various parts or assets into larger, more complex entities

For example, if a house is your component, the neighborhood or city might be assembly models containing multiple group scopes, such as trees and street lights.

#### Model Hierarchy
Prims of the group, assembly, and component kind (and any custom kind inheriting from them) make up the **model hierarchy**. This hierarchy enables better asset organization in your scene, facilitating navigation, asset management, and high-level reasoning about scene structure.

### Python API for Model Kinds

```python
# Construct a Usd.ModelAPI on a prim
prim_model_api = Usd.ModelAPI(prim)

# Return the kind of a prim
prim_model_api.GetKind()

# Set the kind of a prim
prim_model_api.SetKind(kind) 

# Return "true" if the prim represents a model based on its kind metadata
prim_model_api.IsModel()  
```

Model kinds in OpenUSD provide a structured way to organize and manage complex 3D scenes. By defining and adhering to these kinds, artists, designers, and developers can create modular, reusable assets that can be easily combined, referenced, and shared across different projects and workflows.


## Course 6: Learn OpenUSD: Setting Up Basic Animations
**Code:** `openusd/01_Learn_OpenUSD_Setting_Up_Basic_Animations.ipynb`

In OpenUSD, `timeCode` and `timeSample` are two important concepts that enable working with animations and simulation in USD scenes.

### TimeCode
**TimeCode** is a point in time with no unit assigned to it. You can think of these as frames whose units are derived from the stage.

### TimeSample
**TimeSample** refers to the individual time-varying values associated with an attribute in USD. Each attribute can have a collection of timeSamples that map timeCode to the attribute's data type values, allowing for animation over time.

### Time Scaling and Interpolation

In a USD scene, the timeCode ordinates of all timeSamples are scaled to seconds based on the `timeCodesPerSecond` metadata value defined in the root layer. This allows flexibility in encoding timeSamples within a range and scale suitable for the application, while maintaining a robust mapping to real-world time for playback and decoding.

For example, if the root layer has `timeCodesPerSecond=24`, a timeCode value of `48.0` would correspond to 2 seconds (48/24) of real time after timeCode 0.

TimeSamples store time-varying data for attributes, such as positions, rotations, or material properties. When an attribute is evaluated at a specific timeCode, the value is linearly interpolated from the surrounding timeSamples, allowing for smooth animation playback.

### Python API for TimeSamples
```python
# Returns authored TimeSamples
cube.GetDisplayColorAttr().GetTimeSamples()

# Sets TimeSample Value (Gf.Vec3d(0,-4.5,0)) at a specified TimeCode (30)
sphere_xform_api.SetTranslate(Gf.Vec3d(0,-4.5,0), time=Usd.TimeCode(30))

```

## Course 7: Learn OpenUSD: An Introduction to Strength Ordering
**Code:** 
- `openusd/01_Learn_OpenUSD_Intro_to_Strength_Ordering.ipynb`
- `openusd/02_Learn_OpenUSD_Experimenting_With_VariantSets.ipynb`

### Layers and Composition

OpenUSD scenes are organized into **layers**, whose contents can be combined or overridden to create a composed scene. Each layer contains a subset of the complete scene data (such as geometry, materials, or animations).

This layered approach enables **non-destructive editing**, where changes made to one layer do not affect the others. Layers can be thought of as modular pieces that, when composed together, form a complete scene.

A **layer** is a single file or resource that contains scene description data. This could be:
- A USD file (`.usd`, `.usda`, `.usdc`)
- A file format supported by a plugin (e.g., `.gltf`, `.fbx`)
- A resource that's not file-based at all (e.g., from a database)

Each USD stage is made of layer stacks composed at runtime to represent a scene.

While Photoshop composes its final product by compiling each layer on top of the one below it, USD's composition engine combines the data across composition arcs according to a specific **strength ordering**, resolving conflicts based on the arcs' relative strengths. This strength ordering is referred to as **LIVRPS** (pronounced "liver peas") – an acronym we'll explain below.

### Layer Usage

Layers are used for:
- **Separating scene data by discipline** for parallel and modular workstreams
- **Creating reusable asset libraries** that can be referenced across multiple scenes
- **Enabling collaborative workflows** where different teams or artists can work on separate layers concurrently
- **Structuring scene data** for efficient loading and instancing (such as using payloads or references)
- **Versioning and non-destructive editing** by introducing new layers for changes

### Common Layer Functions

The following are common functions we use when interacting with layers in USD:

```python
layer.Reload()  # Clears all content/opinions not saved on that layer
layer.Save()    # Saves content from that layer to disk
```

### Understanding Strength Ordering

![Strength ordering](docs/imgs/strength_ordering.png)

**Strength ordering** governs how opinions and namespaces from multiple layers (or scene sub-hierarchies) are combined, and how conflicts are resolved during scene composition. From the composition, the opinions will be ordered based on their strength and the strongest opinion will take priority.

Strength ordering is the ordered list of composition arcs that determines the precedence of data during composition. When conflicting opinions (data values) exist across layers, the stronger layer's opinion takes precedence, allowing for non-destructive overrides and layering of scene data.

During scene composition, USD's composition engine builds a graph of the layers in the specified strength order. That order is affectionately referred to as the acronym **LIVRPS**, which stands for the list of composition operations, ordered from strongest to weakest: **L**ocal, **I**nherits, **V**ariantSets, **R**eferences, **P**ayloads and **S**pecializes.

### LIVRPS Breakdown (Strongest to Weakest)

#### 1. Local (L)
The algorithm iterates through the **local opinions**. Local opinions are any opinion authored directly on a layer or a sublayer of a layer, without any additional composition.

#### 2. Inherits (I)
**Inherits** allows opinions authored on one source prim in the scenegraph to affect all prims in the scenegraph that author an inherits arc to that source prim. For example, you can make changes to all pine trees in a forest without changing the source of the pine tree itself.

#### 3. Variant Sets (V)
**Variant sets** define one or more scenegraph hierarchies for a prim (called variants), and compose one of them. For example, an object can have multiple geometric representations.

#### 4. References (R) and Payloads (P)
**References** compose the contents of a separate layer as a scenegraph. **Payloads** are similar, but contain the ability to load or unload the layer from the stage at runtime. A typical use of references and payloads would be to bring props into an environment, e.g., furniture in a room.

#### 5. Specializes (S)
**Specializes** is essentially authoring a new fallback value for a property; so if all the other compositional choices result in no value, the specializes value will win. This is commonly used with material libraries - for example, a basic Plastic material may be specialized by a RoughPlastic material which reduces the value on the glossiness property. Any subsequent opinion on the RoughPlastic material will take precedence, because specializes is the weakest composition arc.

### Composition Process

For each prim and property, the engine evaluates the opinions from the layers according to LIVRPS, giving precedence to the stronger layer's opinion when conflicts arise. Stronger layers can override or add to the data defined in weaker layers, enabling non-destructive editing and overrides. LIVRPS is applied recursively - for example, when composing a reference, local opinions within the reference are strongest, followed by inherits, followed by variant sets, etc.

The final composed scene, what we refer to as the **USD stage**, represents the combined data from all layers, with conflicts resolved according to the strength ordering.

### References in Detail

**References** in Universal Scene Description are a composition arc that enable the composition of prims and their descendants onto other prims – this allows us to use references to aggregate larger scenes from smaller units of scene description. This can be done with:
- **External references**: Load data from other files
- **Internal references**: Load data from other parts of the hierarchy

They are fundamental in USD’s composition system, enabling modular and reusable scene description, and they are the second most important composition arc in USD, after sublayers.

A **reference statement** includes:
- The address of the layer to reference from (can be omitted for internal references)
- The prim path to reference (can be omitted if you want to load an entire external layer which has a default prim defined)

When a prim is composed via a reference arc, USD first composes the layer stack of the referenced prim, then adds the resulting prim spec to the destination prim. Then, it applies any overrides or additional composition arcs from the destination prim.

### Python API for References

```python
# Return a UsdReferences object for managing references on a prim
prim.GetReferences()

# Add a reference to the specified asset and prim path
references.AddReference(assetPath, primPath) 

# Remove all references from a prim
references.ClearReferences()
```

### Common Use Cases

References are useful for:
- **Building large, complex scenes** by referencing smaller sub-scenes or components
- **Creating asset libraries** where assets, materials, or other props are reused across several scenes
- **Enabling modular workflows** that support collaborative development and asset management