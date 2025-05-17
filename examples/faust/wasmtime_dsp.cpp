

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

#include "wasmtime_dsp.h"

using namespace std;
using namespace wasmtime;

// ---------------------------------------------------------------------
// Helper to unwrap Wasmtime Result<T,E> and abort on error
// ---------------------------------------------------------------------

template <typename T, typename E>
static T unwrap(Result<T, E> res)
{
    if (res) {
        return res.ok();
    }
    cerr << "Wasmtime error: " << res.err().message() << endl;
    abort();
}

// ---------------------------------------------------------------------
// wasmtime_dsp_factory implementation
// ---------------------------------------------------------------------

static std::vector<uint8_t> readFile(const std::string& path)
{
    std::ifstream is(path, std::ios::binary | std::ios::ate);
    if (!is) {
        throw std::runtime_error("Cannot open " + path);
    }
    std::streamsize size = is.tellg();
    is.seekg(0, std::ios::beg);
    std::vector<uint8_t> buffer(size);
    is.read(reinterpret_cast<char*>(buffer.data()), size);
    return buffer;
}

// Register a single host math function into the linker
template <typename F>
static void def(Linker& linker, const char* name, F&& func)
{
    auto res = linker.func_wrap("env", name, std::forward<F>(func));
    if (!res) {
        cerr << "Failed to register host function " << name << ": " << res.err().message() << endl;
        abort();
    }
}

void wasmtime_dsp_factory::registerMathFuns(Linker& linker)
{
    // Integer
    def(linker, "_abs", static_cast<int32_t (*)(int32_t)>(std::abs));

    // Float unary
    def(linker, "_acosf", static_cast<float (*)(float)>(std::acos));
    def(linker, "_asinf", static_cast<float (*)(float)>(std::asin));
    def(linker, "_atanf", static_cast<float (*)(float)>(std::atan));
    def(linker, "_cosf", static_cast<float (*)(float)>(std::cos));
    def(linker, "_expf", static_cast<float (*)(float)>(std::exp));
    def(linker, "_logf", static_cast<float (*)(float)>(std::log));
    def(linker, "_log10f", static_cast<float (*)(float)>(std::log10));
    def(linker, "_roundf", static_cast<float (*)(float)>(std::round));
    def(linker, "_sinf", static_cast<float (*)(float)>(std::sin));
    def(linker, "_tanf", static_cast<float (*)(float)>(std::tan));

    // Float binary
    def(linker, "_atan2f", static_cast<float (*)(float, float)>(std::atan2));
    def(linker, "_fmodf", static_cast<float (*)(float, float)>(std::fmod));
    def(linker, "_powf", static_cast<float (*)(float, float)>(std::pow));
    def(linker, "_remainderf", static_cast<float (*)(float, float)>(std::remainder));

    // Hyperbolic float
    def(linker, "_acoshf", static_cast<float (*)(float)>(std::acosh));
    def(linker, "_asinhf", static_cast<float (*)(float)>(std::asinh));
    def(linker, "_atanhf", static_cast<float (*)(float)>(std::atanh));
    def(linker, "_coshf", static_cast<float (*)(float)>(std::cosh));
    def(linker, "_sinhf", static_cast<float (*)(float)>(std::sinh));
    def(linker, "_tanhf", static_cast<float (*)(float)>(std::tanh));

    // Double unary
    def(linker, "_acos", static_cast<double (*)(double)>(std::acos));
    def(linker, "_asin", static_cast<double (*)(double)>(std::asin));
    def(linker, "_atan", static_cast<double (*)(double)>(std::atan));
    def(linker, "_cos", static_cast<double (*)(double)>(std::cos));
    def(linker, "_exp", static_cast<double (*)(double)>(std::exp));
    def(linker, "_log", static_cast<double (*)(double)>(std::log));
    def(linker, "_log10", static_cast<double (*)(double)>(std::log10));
    def(linker, "_round", static_cast<double (*)(double)>(std::round));
    def(linker, "_sin", static_cast<double (*)(double)>(std::sin));
    def(linker, "_tan", static_cast<double (*)(double)>(std::tan));

    // Double binary
    def(linker, "_atan2", static_cast<double (*)(double, double)>(std::atan2));
    def(linker, "_fmod", static_cast<double (*)(double, double)>(std::fmod));
    def(linker, "_pow", static_cast<double (*)(double, double)>(std::pow));
    def(linker, "_remainder", static_cast<double (*)(double, double)>(std::remainder));

    // Hyperbolic double
    def(linker, "_acosh", static_cast<double (*)(double)>(std::acosh));
    def(linker, "_asinh", static_cast<double (*)(double)>(std::asinh));
    def(linker, "_atanh", static_cast<double (*)(double)>(std::atanh));
    def(linker, "_cosh", static_cast<double (*)(double)>(std::cosh));
    def(linker, "_sinh", static_cast<double (*)(double)>(std::sinh));
    def(linker, "_tanh", static_cast<double (*)(double)>(std::tanh));
}
wasmtime_dsp_factory::wasmtime_dsp_factory(const std::string& filename)
    : wasm_dsp_factory_imp(), fEngine()
{
    // Prepare runtime objects
    fStore  = std::make_unique<Store>(fEngine);
    fLinker = std::make_unique<Linker>(fEngine);

    // Register math functions
    registerMathFuns(*fLinker);

    // Compile module
    auto bytes = readFile(filename);
    fModule    = unwrap(Module::compile(fEngine, Span<uint8_t>(bytes.data(), bytes.size())));
}

wasmtime_dsp_factory::~wasmtime_dsp_factory() = default;

// ---------------------------------------------------------------------
// Instantiate a new DSP object
// ---------------------------------------------------------------------

dsp* wasmtime_dsp_factory::createDSPInstance()
{
    // Each instance gets its own store so we move‑construct one here
    auto store = std::make_unique<Store>(fEngine);

    // Reuse the already‑configured linker (math imports)
    auto instance = unwrap(fLinker->instantiate(*store, *fModule));

    // Locate exported memory
    auto memExtern = instance.get(*store, "memory");
    if (!memExtern) {
        throw std::runtime_error("Wasm module did not export `memory`!");
    }
    auto  memory  = std::get<Memory>(*memExtern);
    auto  memSpan = memory.data(*store);
    char* basePtr = reinterpret_cast<char*>(memSpan.data());

    return new wasmtime_dsp(this, std::move(store), instance, memory, basePtr);
}

// ---------------------------------------------------------------------
// wasmtime_dsp implementation
// ---------------------------------------------------------------------

wasmtime_dsp::wasmtime_dsp(wasmtime_dsp_factory* factory, std::unique_ptr<Store> store,
                           Instance instance, Memory memory, char* memoryBase)
    : wasm_dsp_imp(factory, memoryBase),
      fStore(std::move(store)),
      fInstance(instance),
      fMemory(memory)
{
    initDecoder();

    // Cache compute function
    auto compute_func = fInstance.get(*fStore, "compute");
    assert(compute_func);
    fComputeFunc = std::get<Func>(*compute_func);
}

wasmtime_dsp::~wasmtime_dsp() = default;

// Macro to call simple exported int->int function with dsp index = 0
#define CALL_INT_EXPORT(name)                                      \
    [&] {                                                          \
        auto ext  = fInstance.get(*fStore, name).value();          \
        auto func = std::get<Func>(ext);                           \
        auto res  = unwrap(func.call(*fStore, {Val(int32_t(0))})); \
        return res[0].i32();                                       \
    }()

int wasmtime_dsp::getNumInputs()
{
    return CALL_INT_EXPORT("getNumInputs");
}

int wasmtime_dsp::getNumOutputs()
{
    return CALL_INT_EXPORT("getNumOutputs");
}

int wasmtime_dsp::getSampleRate()
{
    return CALL_INT_EXPORT("getSampleRate");
}

// Helper to call exported void functions with signature (i32)
void wasmtime_dsp::instanceResetUserInterface()
{
    auto ext  = fInstance.get(*fStore, "instanceResetUserInterface").value();
    auto func = std::get<Func>(ext);
    unwrap(func.call(*fStore, {Val(int32_t(0))}));
}

void wasmtime_dsp::instanceClear()
{
    auto ext  = fInstance.get(*fStore, "instanceClear").value();
    auto func = std::get<Func>(ext);
    unwrap(func.call(*fStore, {Val(int32_t(0))}));
}

// Functions taking sample_rate (i32, i32)
void wasmtime_dsp::init(int sr)
{
    auto func = std::get<Func>(fInstance.get(*fStore, "init").value());
    unwrap(func.call(*fStore, {Val(int32_t(0)), Val(int32_t(sr))}));
}

void wasmtime_dsp::instanceInit(int sr)
{
    auto func = std::get<Func>(fInstance.get(*fStore, "instanceInit").value());
    unwrap(func.call(*fStore, {Val(int32_t(0)), Val(int32_t(sr))}));
}

void wasmtime_dsp::instanceConstants(int sr)
{
    auto func = std::get<Func>(fInstance.get(*fStore, "instanceConstants").value());
    unwrap(func.call(*fStore, {Val(int32_t(0)), Val(int32_t(sr))}));
}

wasmtime_dsp* wasmtime_dsp::clone()
{
    return reinterpret_cast<wasmtime_dsp*>(fFactory->createDSPInstance());
}

// ---------------------------------------------------------------------
// Audio processing
// ---------------------------------------------------------------------

void wasmtime_dsp::compute(int count, FAUSTFLOAT** inputs, FAUSTFLOAT** outputs)
{
    // Copy audio inputs into the shared linear memory
    for (int ch = 0; ch < fFactory->fDecoder->getNumInputs(); ++ch) {
        memcpy(fInputs[ch], inputs[ch], sizeof(FAUSTFLOAT) * count);
    }

    // Call `compute` function
    unwrap(fComputeFunc->call(*fStore, {Val(int32_t(0)), Val(int32_t(count)),
                                        Val(int32_t(fWasmInputs)), Val(int32_t(fWasmOutputs))}));

    // Copy from the shared linear memory into audio outputs
    for (int ch = 0; ch < fFactory->fDecoder->getNumOutputs(); ++ch) {
        memcpy(outputs[ch], fOutputs[ch], sizeof(FAUSTFLOAT) * count);
    }
}
