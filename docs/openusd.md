# OpenUSD
Learn OpenUSD, the core of Omniverse's scene representation format. The main website is [link](https://www.nvidia.com/en-us/learn/learning-path/openusd/), it contains 7 courses in learn Open USD.

## Course1: Learn OpenUSD: Learning About Stages, Prims, and Attributes
[Course Link](https://learn.nvidia.com/courses/course-detail?course_id=course-v1:DLI+S-OV-17+V1)
    - Create and manipulate USD files. Learn how to set up our own USD files from scratch, setting the foundation for your 3D scenes.
    - Define primitives. Get hands-on experience with defining various types of prims, the building blocks of USD, and understand their roles in a 3D environment.
    - Establish scene hierarchies. Discover how to organize and structure 3D elements effectively, creating a coherent and manageable scene hierarchy.
    - Light up scenes. Add dynamic lighting to scenes, bringing them to life and enhancing their visual appeal.
    - Manage attributes and metadata. Delve into the details of attributes and metadata, learning how to set, get, and manipulate these essential elements.
    - Traverse and inspect USD files. Develop skills to traverse through USD files, inspecting and understanding the intricate details of our scenes.
    - Verify prims’ existence. Learn techniques to check for the existence of specific prims, ensuring the integrity and completeness of our 3D scenes.

### Stage
An **OpenUSD stage** presents the scenegraph, which dictates what is in our scene. It is the hierarchy of objects, called prims. These prims can be anything from geometry, to materials, to lights and other organizational elements. This scene is commonly stored in a data structure of connected nodes, which is why we refer to it as the scenegraph.

A stage could be made up entirely with just one USD file (like a robot), or it could be a USD file that includes many more USD files (like a factory with many robots). The stage is the composed result of the file or files that may contribute to a scenegraph. Composition is the result of the algorithm for how all of the USD files (or layers, in USD parlance, as USD content need not be file-backed) should be assembled and combined.

When we leverage OpenUSD stages properly, we can enable:
    - Modularity : Stages enable the modification of individual elements without altering the original files (“non-destructive” editing), fostering a flexible workflow upon modular scene elements.
    - Scalability : Stages can manage large datasets efficiently (e.g., via payloads, which we’ll learn more about when we dive deeper into composition).

```python
# Create a new, empty USD stage where 3D scenes are assembled
Usd.Stage.CreateNew()
  
# Open an existing USD file as a stage
Usd.Stage.Open()
  
# Saves all layers in a USD stage
Usd.Stage.Save()
```
### Hydra
**Hydra**, a powerful rendering architecture within OpenUSD. Understanding Hydra enables efficient and flexible rendering of complex 3D scenes. It bridges the gap between USD and rendering backends, allowing for the development of custom renderers and the integration of various rendering engines. It serves as a bridge between the scene description data, such as USD, and the rendering backend, such as OpenGL or DirectX. The open and extensible nature of Hydra means it supports many different renderers, like Arnold and Renderman, and OpenUSD’s included HdStorm renderer offers a simple way for developers to visualize data out of the box. Hydra supports various render delegate plugins, enabling developers to create custom rendering backends or extend existing ones. OpenUSD ships with HdStorm, the real-time OpenGL/Metal/Vulkan render delegate leveraged by usdview and many other tools. It also ships with HdTiny and HdEmbree, which can be used as examples of how to implement render delegates.

Interaction with Hydra is typically done through the **HydraPython** API, which provides a programmatic interface to control and configure the rendering process.

### USD File Formats
Core OpenUSD file formats – USD, USDA, USDC and USDZ. All the formats are used by OpenUSD for storing and exchanging 3D scene data of various types, including meshes, cameras, lights, and shaders. Developers can interact with USD files using Python bindings.

USDA (.usda) are ASCII text files that encode scene descriptions in a format that can easily be read and edited. It is a native file format used by OpenUSD to store and exchange 3D scene data. It is human-readable, which makes USDA particularly useful for tasks that involve manual editing or inspection of scene data. This makes USDA optimal for small files, such as a stage that is referencing external content.

The Crate Binary Format, or USDC (.usdc), is a compressed binary file format used by OpenUSD to store and exchange 3D scene data. It is designed to minimize load time and provide a more efficient representation of the scene data compared to the human-readable ASCII format (USDA). The Crate Binary Format uses various compression techniques to reduce the file size and improve loading performance. It also employs memory mapping for faster file access and loading times. The structure of the file is organized in a way that allows for efficient parsing and retrieval of the scene data. USDC is extremely efficient for numerically-heavy data, like geometry.

A USD (.usd) file can be either ASCII or binary – the advantage of which is that we can change the underlying format at any point without breaking references. Using USD is also beneficial for debugging, because an asset that is in binary can easily be changed to ASCII to take a look at what might be causing the issue.

USDZ (.usdz). USDZ is an atomic, uncompressed, zipped archive so that we can deliver all of the necessary assets together. We would not use USDZ if we are still making edits to the asset, but it is a great way to package and ship our asset when it is complete.

### Prim
Primitives, or prims for short, are the building blocks of any OpenUSD scene, making understanding them essential for anyone working with 3D content creation and manipulation in the OpenUSD ecosystem. Think of a prim as a container that holds various types of data, attributes, and relationships which define an object or entity within a scene. A prim can be a type of imageable or non-imageable entity, such as a mesh, a material, or a light or an xform. Prims are organized in a hierarchical structure, creating a scenegraph that represents the relationships and transformations between objects in the scene.

Each prim has a unique identifier known as a path, which helps in locating it within the scene graph. For example, a prim’s path might be /World/BuildingA/Geometry/building_geo, indicating that it is a child of the Geometry prim, which itself is a child of the BuildingA prim, and so on.

In Python, working with prims involves several methods using the USD Python API:
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

### scope
In OpenUSD, a scope is a special type of prim that is used primarily as a grouping mechanism in the scenegraph. It does not represent any geometry or renderable content itself but acts as a container for organizing other prims. Think of scope as an empty folder on your computer where you organize files; similarly, scope helps in structuring and organizing prims within a USD scene.

Scope prims are used to create a logical grouping of related prims, which can be particularly useful in complex scenes with numerous elements. For example, a scope might be used to group all prims related to materials, animation, or geometry. A key feature of scopes is that they cannot be transformed, which promotes their usage as lightweight organizational containers. 

When working with scope in USD using Python, a couple functions are particularly useful:
```python
# Used to define a new scope at a specified path on a given stage
UsdGeom.Scope.Define(stage, path)

# This command is generic, but it's useful to confirm that a prim's type is a scope, ensuring correct usage in scripts
prim.IsA(UsdGeom.Scope)
```

### xform
In OpenUSD, an xform is a type of prim that stores transformation data, such as translation, rotation, and scaling, which apply to its child prims. This makes xforms a powerful tool for grouping and manipulating the spatial arrangement of objects in a 3D scene. Xform stands for ‘transform’, reflecting its role in transforming the space in which its children reside.

Working with xform in USD via Python involves several functions:
```python
# Used to define a new Xform prim at a specified path on a given stage
UsdGeom.Xform.Define(stage, path)

# Retrieves the order of transformation operations, which is crucial for understanding how multiple transformations are combined. Different orders can yield different results, so understanding XformOpOrder is important. 
xform.GetXformOpOrderAttr()
	
# Adds a new transform operation to the xform prim, such as translation or rotation, with specified value   
xform.AddXformOp(opType, value)
```

### USD Modules
The USD code repository is made up of four core packages: base, usd, imaging, and usdImaging. To author and read USD data, you only need the base and usd packages. When authoring or querying USD data, you will almost always use a few common USD modules such as Usd, Sdf, and Gf along with some schema modules. Schemas are grouped into schema domains and each domain has its own module. The schema modules you use will depend on the type of scene description you’re working with. For example, UsdGeom for geometry data, UsdShade for materials and shaders, and UsdPhysics for physics scene description.

In Python, you can import these modules from the pxr namespace:
```python
# Import Usd, Sdf, and Gf libraries from Pixar
from pxr import Usd, Sdf, Gf
```
Usd is the core client-facing module for authoring, composing and reading USD. It provides an interface for creating or opening a Stage and generic interfaces for interacting with prims, properties, metadata, and composition arcs.

Sdf (scene description foundation) provides the foundations for serializing scene description to a reference text-based file format and implements scene description layers, (SdfLayer) which stores part of the scene description. Most notably, you will commonly see this module used for managing prim and property paths and creating USD layers.

Gf is the graphics foundation and contains the foundation classes and functions that contribute graphics, like Linear Algebra, Basic Mathematical Operations and Basic Geometry. This module contains classes for 3D data types that you will use for getting and setting particular USD attributes.

USDLux includes a set of light types and light-related schemas. It provides a standardized way to represent various types of lights. Here are a few relevant Python commands for working with USD lights:
```python
# Import the UsdLux module
from pxr import UsdLux
	
# Create a sphere light primitive
UsdLux.SphereLight.Define(stage, '/path/to/light')

# Set the intensity of a light primitive
light_prim.GetIntensityAttr().Set(500)
```

### USD properties: Attributes
USD attributes are one of the two types of USD properties, the other being relationships. Properties are used to describe the characteristics of objects, or “prims,” within a USD scene. Attributes store data values that define the appearance, behavior, or other properties of a prim. Relationships, on the other hand, establish connections between prims, enabling hierarchical structures and data sharing.

To work with attributes in OpenUSD, we will generally use schema-specific APIs. Each schema-specific API has a function to grab its own attributes. Review the following examples to learn more.
```python
# Get the radius value of sphere_prim that is of type UsdGeom.Sphere
sphere_prim.GetRadiusAttr().Get()

# Set the double-sided property of the prim
sphere_prim.GetDoubleSidedAttr().Set(True)

```

### USD properties: Relationships
Relationships establish connections between prims, acting as pointers or links between objects in the scene hierarchy. A relationship allows a prim to target or reference other prims, attributes, or even other relationships. This establishes dependencies between scenegraph objects.
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

Primvars enable efficient management and manipulation of hierarchical object data in complex 3D scenes. Short for primitive variables, primvars are special attributes that contain extra features. They address the following problems in computer graphics:
 - The need to “bind” user data on geometric primitives that becomes available to shaders during rendering.
 - The need to specify a set of values associated with vertices or faces of a primitive that will interpolate across the primitive’s surface under subdivision or shading.
 - The need to inherit attributes down namespace to allow sparse authoring of shareable data.

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

XformCommonAPI is a component of the OpenUSD framework. This API facilitates the authoring and retrieval of a common set of operations with a single translation, rotation, scale and pivot that is generally compatible with import and export into many tools. It’s designed to simplify the interchange of these transformations.

Stage traversal helps us navigate and manipulate the scenegraph more efficiently. We can iterate through child prims, access parent prims, and traverse the hierarchy to find specific prims of interest. This process is facilitated by the stage object, which provides an interface to load, edit, and save USD data. Traversing stages works via the Usd.PrimRange class. There are other methods that use Usd.PrimRange as a base class, such as stage.Traverse.
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

## Course2: Learn OpenUSD: Working With Prims and Default Schemas
[link](https://learn.nvidia.com/courses/course?course_id=course-v1:DLI+S-OV-20+V1&unit=block-v1:DLI+S-OV-20+V1+type@vertical+block@7420568df58a46afb3b4673a0f637f3f)

Specifiers in OpenUSD convey the intent for how a prim or a primSpec should be interpreted in the composed scene. The specifier will be one of three things: Def, Over or Class. Def, which is short for define , defines the prim in the current layer. Def indicates a prim exists, is present on the stage and available for processing.

In OpenUSD, a path is a type that represents the location of a prim within a scene hierarchy. Its string representation consists of a sequence of prim names separated by forward slashes (/), similar to file paths in a directory structure. The stage root, which serves as the starting point for the hierarchy, is represented by a forward slash (/).

For example, the path /World/Geometry/Box represents a prim named Box that is a child of a prim named Geometry, which is a child of the root prim named World.
```python
# Import the Sdf class
from pxr import Sdf

# Return the path of a Usd.Prim as an Sdf.Path object
Usd.Prim.GetPath()

# Retrieve a Usd.Prim at the specified path from the Stage
Usd.Stage.GetPrimAtPath()
```

A default prim in OpenUSD is a top-level prim, or primitive, that is part of the scene’s metadata and serves as the primary entry point or root for a stage. Think of it as the “control point” in the scene, which helps other parts of the system know where to start or what to focus on.

It is best practice to set a default prim in our stages. This is crucial for tools and applications that read USD files, as it guides them to the primary content; for some it may even be considered invalid if the default prim is not specified for the stage. usdchecker checks for a default prim and reports an error if it is not set on a stage. A default prim is also particularly useful when the stage’s root layer is referenced in other stages (such as a reference or payload), as it eliminates the need for consumers to specify a target prim manually.
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

Schemas give meaning to prims in OpenUSD, i.e., “What is this element? What capabilities does it have?”. Schemas define the data models and optional API for encoding and interchanging 3D and non-3D concepts through OpenUSD. Schemas serve as blueprints that author and retrieve data, like attributes and relationships that govern behaviors of elements in a USD scene. They provide a consistent and extensible way to define and interpret data, ensuring data interoperability between different software tools and applications.

## Course3: Learn OpenUSD: Using Attributes
[link](https://learn.nvidia.com/courses/course-detail?course_id=course-v1:DLI+S-OV-19+V1)