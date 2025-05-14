
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

// Adapted from original Wasmer-based implementation to use Wasmtime C++ API
// (https://github.com/bytecodealliance/wasmtime)

#ifndef __wasmtime_dsp__
#define __wasmtime_dsp__

#include <cassert>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// Wasmtime single‑header C++ API
#include "wasmtime.hh"

// FAUST wasm helpers
#include "faust/dsp/wasm-dsp-imp.h"

struct wasmtime_dsp_factory;
class wasmtime_dsp;

/**
 * Factory able to compile a .wasm file produced by the FAUST compiler and
 * instantiate DSP objects running in the Wasmtime runtime.
 */
struct wasmtime_dsp_factory : public wasm_dsp_factory_imp {
    wasmtime::Engine                  fEngine;
    std::unique_ptr<wasmtime::Store>  fStore;
    std::unique_ptr<wasmtime::Linker> fLinker;
    std::optional<wasmtime::Module>   fModule;

    explicit wasmtime_dsp_factory(const std::string& filename);
    virtual ~wasmtime_dsp_factory();

    dsp* createDSPInstance();

    /** Register the math functions that the FAUST-generated module
     *  expects to find in the "env" import namespace.
     */
    static void registerMathFuns(wasmtime::Linker& linker);
};

/**
 * Concrete DSP instance backed by a Wasmtime instance.
 */
class wasmtime_dsp : public wasm_dsp_imp {
   private:
    std::unique_ptr<wasmtime::Store> fStore;
    wasmtime::Instance               fInstance;
    wasmtime::Memory                 fMemory;

    // Cached helpers
    std::optional<wasmtime::Func> fComputeFunc;

   public:
    wasmtime_dsp(wasmtime_dsp_factory* factory, std::unique_ptr<wasmtime::Store> store,
                 wasmtime::Instance instance, wasmtime::Memory memory, char* memoryBase);

    ~wasmtime_dsp() override;

    int getNumInputs() override;
    int getNumOutputs() override;
    int getSampleRate() override;

    void init(int sample_rate) override;
    void instanceInit(int sample_rate) override;
    void instanceConstants(int sample_rate) override;
    void instanceResetUserInterface() override;
    void instanceClear() override;

    wasmtime_dsp* clone() override;  // not supported

    void compute(int count, FAUSTFLOAT** inputs, FAUSTFLOAT** outputs) override;

    void compute(double /*date_usec*/, int count, FAUSTFLOAT** inputs,
                 FAUSTFLOAT** outputs) override
    {
        compute(count, inputs, outputs);
    }
};

#endif /* __wasmtime_dsp__ */
