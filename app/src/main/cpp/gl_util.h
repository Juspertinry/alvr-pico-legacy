#pragma once
// Generic GL helpers shared by every draw module.
#include <GLES3/gl3.h>

// Compile a shader of `type` from source `src`; logs the info log on failure.
// Returns the shader name (0-checked by the caller's link step).
GLuint compile(GLenum type, const char *src);
