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
 FAUST Wasmtime backend using **C API** (Wasmtime v15+)
 ************************************************************************/

#include "wasmtime_dsp.h"

using namespace std;
#define CTX(st) wasmtime_store_context(st)

// ---------------------------------------------------------------------
// Error helpers
// ---------------------------------------------------------------------

static void exit_on_error(const char* context, wasmtime_error_t* error, wasm_trap_t* trap = nullptr)
{
    if (error) {
        wasm_name_t msg;
        wasmtime_error_message(error, &msg);
        cerr << context << ": " << string(msg.data, msg.size) << endl;
        wasm_name_delete(&msg);
        wasmtime_error_delete(error);
        abort();
    }
    if (trap) {
        wasm_name_t msg;
        wasm_trap_message(trap, &msg);
        cerr << context << ": trap – " << string(msg.data, msg.size) << endl;
        wasm_name_delete(&msg);
        wasm_trap_delete(trap);
        abort();
    }
}

// ---------------------------------------------------------------------
// Functype helpers
// ---------------------------------------------------------------------

// helper – build 1-param/1-result functype safely
static wasm_functype_t* fn_1_1(wasm_valkind_t p, wasm_valkind_t r)
{
    // allocate the pointer array on the heap (linker will free it later)
    auto** params  = new wasm_valtype_t*[1];
    auto** results = new wasm_valtype_t*[1];
    params[0]      = wasm_valtype_new(p);
    results[0]     = wasm_valtype_new(r);

    wasm_valtype_vec_t pvec;
    wasm_valtype_vec_t rvec;
    wasm_valtype_vec_new(&pvec, 1, params);   // takes ownership of `params`
    wasm_valtype_vec_new(&rvec, 1, results);  // takes ownership of `results`

    return wasm_functype_new(&pvec, &rvec);  // takes ownership of valtypes & arrays
}

// helper – 2-param/1-result
static wasm_functype_t* fn_2_1(wasm_valkind_t p0, wasm_valkind_t p1, wasm_valkind_t r0)
{
    auto** params  = new wasm_valtype_t*[2];
    auto** results = new wasm_valtype_t*[1];
    params[0]      = wasm_valtype_new(p0);
    params[1]      = wasm_valtype_new(p1);
    results[0]     = wasm_valtype_new(r0);

    wasm_valtype_vec_t pvec;
    wasm_valtype_vec_t rvec;
    wasm_valtype_vec_new(&pvec, 2, params);
    wasm_valtype_vec_new(&rvec, 1, results);

    return wasm_functype_new(&pvec, &rvec);
}

// ---------------------------------------------------------------------
// Host math callbacks
// ---------------------------------------------------------------------

#define UNARY_CB(NAME, FN, VKIND)                                                                \
    static wasm_trap_t* NAME##_cb(void*, wasmtime_caller_t*, const wasmtime_val_t* args, size_t, \
                                  wasmtime_val_t* res, size_t)                                   \
    {                                                                                            \
        res[0].kind = VKIND;                                                                     \
        if (VKIND == WASM_F32)                                                                   \
            res[0].of.f32 = FN(args[0].of.f32);                                                  \
        else                                                                                     \
            res[0].of.f64 = FN(args[0].of.f64);                                                  \
        return nullptr;                                                                          \
    }

#define BINARY_CB(NAME, FN, VKIND)                                                               \
    static wasm_trap_t* NAME##_cb(void*, wasmtime_caller_t*, const wasmtime_val_t* args, size_t, \
                                  wasmtime_val_t* res, size_t)                                   \
    {                                                                                            \
        res[0].kind = VKIND;                                                                     \
        if (VKIND == WASM_F32)                                                                   \
            res[0].of.f32 = FN(args[0].of.f32, args[1].of.f32);                                  \
        else                                                                                     \
            res[0].of.f64 = FN(args[0].of.f64, args[1].of.f64);                                  \
        return nullptr;                                                                          \
    }

UNARY_CB(_sinf, std::sinf, WASM_F32)
UNARY_CB(_cosf, std::cosf, WASM_F32)
UNARY_CB(_tanf, std::tanf, WASM_F32)
UNARY_CB(_asinf, std::asinf, WASM_F32)
UNARY_CB(_acosf, std::acosf, WASM_F32)
UNARY_CB(_atanf, std::atanf, WASM_F32)
UNARY_CB(_expf, std::expf, WASM_F32)
UNARY_CB(_logf, std::logf, WASM_F32)
UNARY_CB(_log10f, std::log10f, WASM_F32)
UNARY_CB(_roundf, std::roundf, WASM_F32)
UNARY_CB(_sinhf, std::sinhf, WASM_F32)
UNARY_CB(_coshf, std::coshf, WASM_F32)
UNARY_CB(_tanhf, std::tanhf, WASM_F32)
UNARY_CB(_asinhf, std::asinhf, WASM_F32)
UNARY_CB(_acoshf, std::acoshf, WASM_F32)
UNARY_CB(_atanhf, std::atanhf, WASM_F32)

UNARY_CB(_sin, std::sin, WASM_F64)
UNARY_CB(_cos, std::cos, WASM_F64)
UNARY_CB(_tan, std::tan, WASM_F64)
UNARY_CB(_asin, std::asin, WASM_F64)
UNARY_CB(_acos, std::acos, WASM_F64)
UNARY_CB(_atan, std::atan, WASM_F64)
UNARY_CB(_exp, std::exp, WASM_F64)
UNARY_CB(_log, std::log, WASM_F64)
UNARY_CB(_log10, std::log10, WASM_F64)
UNARY_CB(_round, std::round, WASM_F64)
UNARY_CB(_sinh, std::sinh, WASM_F64)
UNARY_CB(_cosh, std::cosh, WASM_F64)
UNARY_CB(_tanh, std::tanh, WASM_F64)
UNARY_CB(_asinh, std::asinh, WASM_F64)
UNARY_CB(_acosh, std::acosh, WASM_F64)
UNARY_CB(_atanh, std::atanh, WASM_F64)

BINARY_CB(_atan2f, std::atan2f, WASM_F32)
BINARY_CB(_fmodf, std::fmodf, WASM_F32)
BINARY_CB(_powf, std::powf, WASM_F32)
BINARY_CB(_remainderf, std::remainderf, WASM_F32)

BINARY_CB(_atan2, std::atan2, WASM_F64)
BINARY_CB(_fmod, std::fmod, WASM_F64)
BINARY_CB(_pow, std::pow, WASM_F64)
BINARY_CB(_remainder, std::remainder, WASM_F64)

// abs(int)
static wasm_trap_t* _abs_cb(void*, wasmtime_caller_t*, const wasmtime_val_t* args, size_t,
                            wasmtime_val_t* res, size_t)
{
    res[0].kind   = WASM_I32;
    res[0].of.i32 = std::abs(args[0].of.i32);
    return nullptr;
}

// ---------------------------------------------------------------------
// File read helper
// ---------------------------------------------------------------------

static vector<uint8_t> readFile(const string& path)
{
    ifstream is(path, ios::binary | ios::ate);
    if (!is) {
        throw runtime_error("Cannot open " + path);
    }
    streamsize sz = is.tellg();
    is.seekg(0, ios::beg);
    vector<uint8_t> buf(sz);
    is.read(reinterpret_cast<char*>(buf.data()), sz);
    return buf;
}

// ---------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------

wasmtime_dsp_factory::wasmtime_dsp_factory(const string& wasmFile) : wasm_dsp_factory_imp()
{
    fEngine = wasm_engine_new();
    fLinker = wasmtime_linker_new(fEngine);

    // Register math funcs
    wasmtime_store_t* tmp = wasmtime_store_new(fEngine, nullptr, nullptr);
    registerMathFuns(fLinker, tmp);

    // Compile module
    auto bytes = readFile(wasmFile);
    wasm_byte_vec_new_uninitialized(&fWasmBytes, bytes.size());
    memcpy(fWasmBytes.data, bytes.data(), bytes.size());

    wasmtime_error_t* err = wasmtime_module_new(
        fEngine, reinterpret_cast<const uint8_t*>(fWasmBytes.data), fWasmBytes.size, &fModule);
    exit_on_error("compile module", err);

    wasmtime_store_delete(tmp);
}

wasmtime_dsp_factory::~wasmtime_dsp_factory()
{
    if (fModule) {
        wasmtime_module_delete(fModule);
    }
    if (fLinker) {
        wasmtime_linker_delete(fLinker);
    }
    if (fEngine) {
        wasm_engine_delete(fEngine);
    }
    wasm_byte_vec_delete(&fWasmBytes);
}

//--------------------------------------------------------------------
// Helper: register one host callback
//--------------------------------------------------------------------
static void define_func(wasmtime_linker_t* linker,
                        wasmtime_store_t* /*unused – keeps the old call-sites happy*/,
                        const char*              name,
                        wasm_functype_t*         ft,  // **non-const**
                        wasmtime_func_callback_t cb)
{
    wasmtime_error_t* err = wasmtime_linker_define_func(linker, "env", 3,    // module name
                                                        name, strlen(name),  // export name
                                                        ft,                  // type
                                                        cb,                  // callback
                                                        nullptr,             // env  (not needed)
                                                        nullptr  // finalizer (not needed)
    );
    exit_on_error("define callback", err);
}

void wasmtime_dsp_factory::registerMathFuns(wasmtime_linker_t* linker, wasmtime_store_t* s)
{
    define_func(linker, s, "_abs", fn_1_1(WASM_I32, WASM_I32), _abs_cb);

#define REG_UN(name, kind) define_func(linker, s, #name, fn_1_1(kind, kind), name##_cb)
#define REG_BIN(name, kind) define_func(linker, s, #name, fn_2_1(kind, kind, kind), name##_cb)

    // Float
    REG_UN(_sinf, WASM_F32);
    REG_UN(_cosf, WASM_F32);
    REG_UN(_tanf, WASM_F32);
    REG_UN(_asinf, WASM_F32);
    REG_UN(_acosf, WASM_F32);
    REG_UN(_atanf, WASM_F32);
    REG_UN(_expf, WASM_F32);
    REG_UN(_logf, WASM_F32);
    REG_UN(_log10f, WASM_F32);
    REG_UN(_roundf, WASM_F32);
    REG_UN(_sinhf, WASM_F32);
    REG_UN(_coshf, WASM_F32);
    REG_UN(_tanhf, WASM_F32);
    REG_UN(_asinhf, WASM_F32);
    REG_UN(_acoshf, WASM_F32);
    REG_UN(_atanhf, WASM_F32);

    REG_BIN(_atan2f, WASM_F32);
    REG_BIN(_fmodf, WASM_F32);
    REG_BIN(_powf, WASM_F32);
    REG_BIN(_remainderf, WASM_F32);

    // Double
    REG_UN(_sin, WASM_F64);
    REG_UN(_cos, WASM_F64);
    REG_UN(_tan, WASM_F64);
    REG_UN(_asin, WASM_F64);
    REG_UN(_acos, WASM_F64);
    REG_UN(_atan, WASM_F64);
    REG_UN(_exp, WASM_F64);
    REG_UN(_log, WASM_F64);
    REG_UN(_log10, WASM_F64);
    REG_UN(_round, WASM_F64);
    REG_UN(_sinh, WASM_F64);
    REG_UN(_cosh, WASM_F64);
    REG_UN(_tanh, WASM_F64);
    REG_UN(_asinh, WASM_F64);
    REG_UN(_acosh, WASM_F64);
    REG_UN(_atanh, WASM_F64);

    REG_BIN(_atan2, WASM_F64);
    REG_BIN(_fmod, WASM_F64);
    REG_BIN(_pow, WASM_F64);
    REG_BIN(_remainder, WASM_F64);

#undef REG_UN
#undef REG_BIN
}

dsp* wasmtime_dsp_factory::createDSPInstance()
{
    wasmtime_store_t* store = wasmtime_store_new(fEngine, nullptr, nullptr);

    wasm_trap_t*        trap = nullptr;
    wasmtime_instance_t inst;
    wasmtime_error_t* err = wasmtime_linker_instantiate(fLinker, CTX(store), fModule, &inst, &trap);
    exit_on_error("instantiate", err, trap);

    wasmtime_extern_t memExt;
    bool ok = wasmtime_instance_export_get(CTX(store), &inst, "memory", strlen("memory"), &memExt);
    if (!ok || memExt.kind != WASMTIME_EXTERN_MEMORY) {
        throw runtime_error("memory export missing");
    }

    char* base = reinterpret_cast<char*>(wasmtime_memory_data(CTX(store), &memExt.of.memory));

    return new wasmtime_dsp(this, store, inst, memExt.of.memory, base);
}

// ---------------------------------------------------------------------
// DSP instance
// ---------------------------------------------------------------------

wasmtime_dsp::wasmtime_dsp(wasmtime_dsp_factory* fac, wasmtime_store_t* st,
                           const wasmtime_instance_t& inst, const wasmtime_memory_t& mem,
                           char* base)
    : wasm_dsp_imp(fac, base), fStore(st), fInstance(inst), fMemory(mem), fMemoryBase(base)
{
    initDecoder();

    wasmtime_extern_t ext;
    bool              ok =
        wasmtime_instance_export_get(CTX(fStore), &fInstance, "compute", strlen("compute"), &ext);
    assert(ok && ext.kind == WASMTIME_EXTERN_FUNC);
    fComputeFunc = ext.of.func;
}

wasmtime_dsp::~wasmtime_dsp()
{
    wasmtime_store_delete(fStore);
}

int wasmtime_dsp::callIntExport(const char* name)
{
    wasmtime_extern_t ext;
    bool ok = wasmtime_instance_export_get(CTX(fStore), &fInstance, name, strlen(name), &ext);
    if (!ok || ext.kind != WASMTIME_EXTERN_FUNC) {
        return -1;
    }

    wasmtime_val_t    arg{.kind = WASM_I32, .of = {.i32 = 0}};
    wasmtime_val_t    out;
    wasm_trap_t*      trap = nullptr;
    wasmtime_error_t* err  = wasmtime_func_call(CTX(fStore), &ext.of.func, &arg, 1, &out, 1, &trap);
    exit_on_error(name, err, trap);
    return out.of.i32;
}

int wasmtime_dsp::getNumInputs()
{
    return callIntExport("getNumInputs");
}
int wasmtime_dsp::getNumOutputs()
{
    return callIntExport("getNumOutputs");
}
int wasmtime_dsp::getSampleRate()
{
    return callIntExport("getSampleRate");
}

void wasmtime_dsp::instanceResetUserInterface()
{
    wasmtime_extern_t ext;
    bool ok = wasmtime_instance_export_get(CTX(fStore), &fInstance, "instanceResetUserInterface",
                                           strlen("instanceResetUserInterface"), &ext);
    if (!ok || ext.kind != WASMTIME_EXTERN_FUNC) {
        return;
    }

    wasmtime_val_t    arg{.kind = WASM_I32, .of = {.i32 = 0}};
    wasm_trap_t*      trap = nullptr;
    wasmtime_error_t* err =
        wasmtime_func_call(CTX(fStore), &ext.of.func, &arg, 1, nullptr, 0, &trap);
    exit_on_error("instanceResetUI", err, trap);
}

void wasmtime_dsp::instanceClear()
{
    wasmtime_extern_t ext;
    bool              ok = wasmtime_instance_export_get(CTX(fStore), &fInstance, "instanceClear",
                                                        strlen("instanceClear"), &ext);
    if (!ok || ext.kind != WASMTIME_EXTERN_FUNC) {
        return;
    }

    wasmtime_val_t    arg{.kind = WASM_I32, .of = {.i32 = 0}};
    wasm_trap_t*      trap = nullptr;
    wasmtime_error_t* err =
        wasmtime_func_call(CTX(fStore), &ext.of.func, &arg, 1, nullptr, 0, &trap);
    exit_on_error("instanceClear", err, trap);
}

static void call_i32_i32(wasmtime_store_t* st, const wasmtime_instance_t* inst, const char* fn,
                         int v, wasm_trap_t** trap)
{
    wasmtime_extern_t ext;
    bool              ok = wasmtime_instance_export_get(CTX(st), inst, fn, strlen(fn), &ext);
    if (!ok || ext.kind != WASMTIME_EXTERN_FUNC) {
        throw runtime_error(string("missing export ") + fn);
    }

    wasmtime_val_t a[2];
    a[0].kind             = WASM_I32;
    a[0].of.i32           = 0;
    a[1].kind             = WASM_I32;
    a[1].of.i32           = v;
    wasmtime_error_t* err = wasmtime_func_call(CTX(st), &ext.of.func, a, 2, nullptr, 0, trap);
    exit_on_error(fn, err, *trap);
}

void wasmtime_dsp::init(int sr)
{
    wasm_trap_t* t = nullptr;
    call_i32_i32(fStore, &fInstance, "init", sr, &t);
}
void wasmtime_dsp::instanceInit(int sr)
{
    wasm_trap_t* t = nullptr;
    call_i32_i32(fStore, &fInstance, "instanceInit", sr, &t);
}
void wasmtime_dsp::instanceConstants(int sr)
{
    wasm_trap_t* t = nullptr;
    call_i32_i32(fStore, &fInstance, "instanceConstants", sr, &t);
}

wasmtime_dsp* wasmtime_dsp::clone()
{
    return reinterpret_cast<wasmtime_dsp*>(fFactory->createDSPInstance());
}

void wasmtime_dsp::compute(int count, FAUSTFLOAT** inputs, FAUSTFLOAT** outputs)
{
    for (int c = 0; c < fFactory->fDecoder->getNumInputs(); ++c) {
        memcpy(fInputs[c], inputs[c], sizeof(FAUSTFLOAT) * count);
    }

    wasmtime_val_t a[4];
    a[0].kind   = WASM_I32;
    a[0].of.i32 = 0;
    a[1].kind   = WASM_I32;
    a[1].of.i32 = count;
    a[2].kind   = WASM_I32;
    a[2].of.i32 = fWasmInputs;
    a[3].kind   = WASM_I32;
    a[3].of.i32 = fWasmOutputs;

    wasm_trap_t*      trap = nullptr;
    wasmtime_error_t* err = wasmtime_func_call(CTX(fStore), &fComputeFunc, a, 4, nullptr, 0, &trap);
    exit_on_error("compute", err, trap);

    for (int c = 0; c < fFactory->fDecoder->getNumOutputs(); ++c) {
        memcpy(outputs[c], fOutputs[c], sizeof(FAUSTFLOAT) * count);
    }
}
