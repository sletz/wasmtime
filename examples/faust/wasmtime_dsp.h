
/************************************************************************
 FAUST Architecture File
 Copyright (C) 2025 GRAME, Centre National de Creation Musicale
 ---------------------------------------------------------------------
 This Architecture section is free software; you can redistribute it
 and/or modify it under the terms of the GNU General Public License
 as published by the Free Software Foundation; either version 3 of
 the License, or (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; If not, see <http://www.gnu.org/licenses/>.

 EXCEPTION : As a special exception, you may create a larger work
 that contains this FAUST architecture section and distribute
 that work under terms of your choice, so long as this FAUST
 architecture section is not modified.
 ************************************************************************/

/************************************************************************
  Rewritten to use the Wasmtime **C** API (v15+) rather than the C++ API.
 ************************************************************************/

#ifndef __wasmtime_dsp__
#define __wasmtime_dsp__

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <wasm.h>
#include <wasmtime.h>

#include "faust/dsp/wasm-dsp-imp.h"

/**
 * Forward declarations
 */
struct wasmtime_dsp_factory;
class wasmtime_dsp;

/**
 * Factory able to compile a .wasm file produced by the FAUST compiler and
 * instantiate DSP objects running in the Wasmtime runtime (C‑API flavour).
 */
struct wasmtime_dsp_factory : public wasm_dsp_factory_imp {
    // Long‑lived objects shared by all DSP instances
    wasm_engine_t*     fEngine    = nullptr;
    wasmtime_linker_t* fLinker    = nullptr;
    wasm_byte_vec_t    fWasmBytes = WASM_EMPTY_VEC;
    wasmtime_module_t* fModule    = nullptr;

    explicit wasmtime_dsp_factory(const std::string& filename);
    ~wasmtime_dsp_factory() override;

    dsp* createDSPInstance() override;

    /** Register the math functions that the FAUST‑generated WASM expects. */
    static void registerMathFuns(wasmtime_linker_t* linker, wasmtime_store_t* store);
};

/**
 * Concrete DSP instance backed by a Wasmtime instance (C‑API flavour).
 */
class wasmtime_dsp : public wasm_dsp_imp {
   private:
    // One store per DSP instance (required for thread‑safety)
    wasmtime_store_t*   fStore = nullptr;
    wasmtime_instance_t fInstance{};
    wasmtime_memory_t   fMemory{};
    wasmtime_func_t     fComputeFunc{};
    char*               fMemoryBase = nullptr;

    /** Helper – call exported int(int) with dsp index = 0 */
    int callIntExport(const char* name);

   public:
    wasmtime_dsp(wasmtime_dsp_factory* factory, wasmtime_store_t* store,
                 const wasmtime_instance_t& instance, const wasmtime_memory_t& memory,
                 char* memoryBase);

    ~wasmtime_dsp() override;

    // DSP meta‑data ----------------------------------------------------------------
    int getNumInputs() override;
    int getNumOutputs() override;
    int getSampleRate() override;

    // Life‑cycle -------------------------------------------------------------------
    void init(int sample_rate) override;
    void instanceInit(int sample_rate) override;
    void instanceConstants(int sample_rate) override;
    void instanceResetUserInterface() override;
    void instanceClear() override;

    wasmtime_dsp* clone() override;  // simple re‑instantiate

    // Audio processing -------------------------------------------------------------
    void compute(int count, FAUSTFLOAT** inputs, FAUSTFLOAT** outputs) override;
    void compute(double date_usec, int count, FAUSTFLOAT** inputs, FAUSTFLOAT** outputs) override
    {
        compute(count, inputs, outputs);
    }
};

#endif /* __wasmtime_dsp__ */
