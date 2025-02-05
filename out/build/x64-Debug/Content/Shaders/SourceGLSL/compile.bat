for %%f in (*.vert.glsl) do (
    if exist "%%f" (
        glslc.exe -fshader-stage=vertex "%%f" -o "..\Compiled\SPIRV\%%~nf.spv"
    )
)

for %%f in (*.frag.glsl) do (
    if exist "%%f" (
        glslc.exe -fshader-stage=fragment "%%f" -o "..\Compiled\SPIRV\%%~nf.spv"
    )
)

for %%f in (*.comp.glsl) do (
    if exist "%%f" (
        glslc.exe -fshader-stage=compute "%%f" -o "..\Compiled\SPIRV\%%~nf.spv"
    )
)