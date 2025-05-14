

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
    def(linker, "_abs", [](int32_t v) { return (int32_t)std::abs(v); });

    // Float unary
    def(linker, "_acosf", [](float v) { return std::acosf(v); });
    def(linker, "_asinf", [](float v) { return std::asinf(v); });
    def(linker, "_atanf", [](float v) { return std::atanf(v); });
    def(linker, "_cosf", [](float v) { return std::cosf(v); });
    def(linker, "_expf", [](float v) { return std::expf(v); });
    def(linker, "_logf", [](float v) { return std::logf(v); });
    def(linker, "_log10f", [](float v) { return std::log10f(v); });
    def(linker, "_roundf", [](float v) { return std::roundf(v); });
    def(linker, "_sinf", [](float v) { return std::sinf(v); });
    def(linker, "_tanf", [](float v) { return std::tanf(v); });

    // Float binary
    def(linker, "_atan2f", [](float a, float b) { return std::atan2f(a, b); });
    def(linker, "_fmodf", [](float a, float b) { return std::fmodf(a, b); });
    def(linker, "_powf", [](float a, float b) { return std::powf(a, b); });
    def(linker, "_remainderf", [](float a, float b) { return std::remainderf(a, b); });

    // Hyperbolic float
    def(linker, "_acoshf", [](float v) { return std::acoshf(v); });
    def(linker, "_asinhf", [](float v) { return std::asinhf(v); });
    def(linker, "_atanhf", [](float v) { return std::atanhf(v); });
    def(linker, "_coshf", [](float v) { return std::coshf(v); });
    def(linker, "_sinhf", [](float v) { return std::sinhf(v); });
    def(linker, "_tanhf", [](float v) { return std::tanhf(v); });

    // Double unary
    def(linker, "_acos", [](double v) { return std::acos(v); });
    def(linker, "_asin", [](double v) { return std::asin(v); });
    def(linker, "_atan", [](double v) { return std::atan(v); });
    def(linker, "_cos", [](double v) { return std::cos(v); });
    def(linker, "_exp", [](double v) { return std::exp(v); });
    def(linker, "_log", [](double v) { return std::log(v); });
    def(linker, "_log10", [](double v) { return std::log10(v); });
    def(linker, "_round", [](double v) { return std::round(v); });
    def(linker, "_sin", [](double v) { return std::sin(v); });
    def(linker, "_tan", [](double v) { return std::tan(v); });

    // Double binary
    def(linker, "_atan2", [](double a, double b) { return std::atan2(a, b); });
    def(linker, "_fmod", [](double a, double b) { return std::fmod(a, b); });
    def(linker, "_pow", [](double a, double b) { return std::pow(a, b); });
    def(linker, "_remainder", [](double a, double b) { return std::remainder(a, b); });

    // Hyperbolic double
    def(linker, "_acosh", [](double v) { return std::acosh(v); });
    def(linker, "_asinh", [](double v) { return std::asinh(v); });
    def(linker, "_atanh", [](double v) { return std::atanh(v); });
    def(linker, "_cosh", [](double v) { return std::cosh(v); });
    def(linker, "_sinh", [](double v) { return std::sinh(v); });
    def(linker, "_tanh", [](double v) { return std::tanh(v); });
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

// Macro to call simple exported int->int function with dsp index=0
#define CALL_INT_EXPORT(name)                                           \
    [&] {                                                               \
        auto             ext    = fInstance.get(*fStore, name).value(); \
        auto             func   = std::get<Func>(ext);                  \
        std::vector<Val> params = {Val(int32_t(0))};                    \
        auto             res    = unwrap(func.call(*fStore, params));   \
        return res[0].i32();                                            \
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
static void call_sr(Func& func, Store& store, int sr)
{
    unwrap(func.call(store, {Val(int32_t(0)), Val(int32_t(sr))}));
}

void wasmtime_dsp::init(int sr)
{
    auto func = std::get<Func>(fInstance.get(*fStore, "init").value());
    call_sr(func, *fStore, sr);
}

void wasmtime_dsp::instanceInit(int sr)
{
    auto func = std::get<Func>(fInstance.get(*fStore, "instanceInit").value());
    call_sr(func, *fStore, sr);
}

void wasmtime_dsp::instanceConstants(int sr)
{
    auto func = std::get<Func>(fInstance.get(*fStore, "instanceConstants").value());
    call_sr(func, *fStore, sr);
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
        std::memcpy(fInputs[ch], inputs[ch], sizeof(FAUSTFLOAT) * count);
    }

    // Call `compute` function
    unwrap(fComputeFunc->call(*fStore, {Val(int32_t(0)), Val(int32_t(count)),
                                        Val(int32_t(fWasmInputs)), Val(int32_t(fWasmOutputs))}));

    // Copy from the shared linear memory into audio outputs
    for (int ch = 0; ch < fFactory->fDecoder->getNumOutputs(); ++ch) {
        std::memcpy(outputs[ch], fOutputs[ch], sizeof(FAUSTFLOAT) * count);
    }
}
