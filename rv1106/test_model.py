#!/usr/bin/env python3
"""
Test the recurrent policy .rknn model on RV1106, all tensors int16:
    obs[32] + h_in[512]  ->  actions[6] + h_out[512]

Feeds random int16 obs, carries the hidden state from one run to the next
(h_out -> h_in, like the real control loop), prints input -> output, and times
the inference in ms. Standard library only (ctypes -> librknnmrt.so).

    python3 test_model.py                         # uses MODEL_PATH below
    python3 test_model.py my_model.rknn --runs 1000 --seed 0
    python3 test_model.py --hidden zero           # h_in = 0 every run
"""
import argparse
import array
import ctypes as C
import gc
import os
import random
import time

# ============================== config ==============================

HERE = os.path.dirname(os.path.abspath(__file__))

# the .rknn file to test; default: models/ next to this script
MODEL_PATH = os.path.join(HERE, "models", "beni512_dfp16.rknn")

# librknnmrt.so is the RKNN runtime (Rockchip's C library that drives the NPU);
# Python calls it through ctypes. The first path that loads is used.
LIB_PATHS = ["/oem/usr/lib/librknnmrt.so",     # Luckfox / RV1106 SDK images
             "/usr/lib/librknnmrt.so",
             "librknnmrt.so"]                  # anywhere on LD_LIBRARY_PATH

# what this model must have (tensors are told apart by size)
N_OBS = 32
N_HIDDEN = 512
N_ACTIONS = 6

# random input ranges, real units: the min/max RKNN-Toolkit saw for each
# input while quantizing (quant_tab in the .rknn). The .rknn has NO normalizer
# inside (the .onnx does): real obs must be (obs - mean) / divisor from
# models/beni512_19500_normalizer.json first, as run_policy.py does
OBS_RANGE = (-18.06, 12.43)
HIDDEN_RANGE = (-1.0, 1.0)

# ============================ rknn_api.h ============================

RKNN_QUERY_IN_OUT_NUM = 0
RKNN_QUERY_INPUT_ATTR = 1
RKNN_QUERY_OUTPUT_ATTR = 2
RKNN_QUERY_SDK_VERSION = 5

RKNN_MEMORY_SYNC_TO_DEVICE = 0x1
RKNN_MEMORY_SYNC_FROM_DEVICE = 0x2

RKNN_TENSOR_INT16 = 4
TENSOR_TYPES = {0: "float32", 1: "float16", 2: "int8", 3: "uint8", 4: "int16",
                5: "uint16", 6: "int32", 7: "uint32", 8: "int64", 9: "bool", 10: "int4"}
QNT_NONE, QNT_DFP, QNT_AFFINE = 0, 1, 2

INT16_MIN, INT16_MAX = -32768, 32767

# rknn_context is uint32 on 32-bit ARM (RV1106), uint64 on 64-bit
Context = C.c_uint32 if C.sizeof(C.c_void_p) == 4 else C.c_uint64


class IoNum(C.Structure):
    _fields_ = [("n_input", C.c_uint32), ("n_output", C.c_uint32)]


class SdkVersion(C.Structure):
    _fields_ = [("api_version", C.c_char * 256), ("drv_version", C.c_char * 256)]


class TensorAttr(C.Structure):
    _fields_ = [
        ("index", C.c_uint32),
        ("n_dims", C.c_uint32),
        ("dims", C.c_uint32 * 16),
        ("name", C.c_char * 256),
        ("n_elems", C.c_uint32),
        ("size", C.c_uint32),
        ("fmt", C.c_int),
        ("type", C.c_int),
        ("qnt_type", C.c_int),
        ("fl", C.c_int8),
        ("zp", C.c_int32),
        ("scale", C.c_float),
        ("w_stride", C.c_uint32),
        ("size_with_stride", C.c_uint32),
        ("pass_through", C.c_uint8),
        ("h_stride", C.c_uint32),
    ]


class TensorMem(C.Structure):
    _fields_ = [
        ("virt_addr", C.c_void_p),
        ("phys_addr", C.c_uint64),
        ("fd", C.c_int32),
        ("offset", C.c_int32),
        ("size", C.c_uint32),
        ("flags", C.c_uint32),
        ("priv_data", C.c_void_p),
    ]

# ============================== tensor ==============================


class Tensor:
    """One int16 input or output and its zero-copy NPU buffer."""

    def __init__(self, attr, mem):
        self.attr = attr
        self.mem = mem
        self.n = attr.n_elems
        if attr.qnt_type == QNT_AFFINE:
            self.scale, self.zp = attr.scale, attr.zp
        elif attr.qnt_type == QNT_DFP:
            self.scale, self.zp = 2.0 ** (-attr.fl), 0
        else:
            self.scale, self.zp = 1.0, 0

    @property
    def name(self):
        return self.attr.name.decode(errors="replace").rstrip("\0")

    @property
    def shape(self):
        return list(self.attr.dims[:self.attr.n_dims])

    @property
    def type_name(self):
        return TENSOR_TYPES.get(self.attr.type, str(self.attr.type))

    def describe(self):
        a = self.attr
        text = "%-20s shape=%s %s" % (self.name, self.shape, self.type_name)
        if a.qnt_type == QNT_DFP:
            text += " (fixed point fl=%d: real = raw / %d)" % (a.fl, 2 ** a.fl)
        elif a.qnt_type == QNT_AFFINE:
            text += " (real = (raw - %d) * %g)" % (self.zp, self.scale)
        return text

    def write(self, raw):
        """int16 values -> NPU buffer, as is."""
        data = array.array("h", raw).tobytes()
        C.memmove(self.mem.contents.virt_addr, data, len(data))

    def read(self):
        """NPU buffer -> int16 values, as is."""
        raw = array.array("h")
        raw.frombytes(C.string_at(self.mem.contents.virt_addr, self.n * 2))
        return raw.tolist()

    def to_real(self, raw):
        return [(v - self.zp) * self.scale for v in raw]

    def to_raw(self, real):
        """one real value -> int16, clamped"""
        return max(INT16_MIN, min(INT16_MAX, round(real / self.scale) + self.zp))

# =============================== model ==============================


def load_lib(paths):
    errors = []
    for path in paths:
        try:
            return C.CDLL(path), path
        except OSError as e:
            errors.append("  %s: %s" % (path, e))
    raise RuntimeError("could not load librknnmrt.so:\n" + "\n".join(errors))


def check(code, label):
    if code != 0:
        raise RuntimeError("%s failed: %d" % (label, code))


class RKNNModel:
    def __init__(self, model_path, lib_paths=LIB_PATHS):
        self.lib, self.lib_path = load_lib(lib_paths)
        lib = self.lib
        lib.rknn_init.argtypes = [C.POINTER(Context), C.c_void_p, C.c_uint32, C.c_uint32, C.c_void_p]
        lib.rknn_query.argtypes = [Context, C.c_int, C.c_void_p, C.c_uint32]
        lib.rknn_run.argtypes = [Context, C.c_void_p]
        lib.rknn_destroy.argtypes = [Context]
        lib.rknn_create_mem.argtypes = [Context, C.c_uint32]
        lib.rknn_create_mem.restype = C.POINTER(TensorMem)
        lib.rknn_set_io_mem.argtypes = [Context, C.POINTER(TensorMem), C.POINTER(TensorAttr)]
        lib.rknn_mem_sync.argtypes = [Context, C.POINTER(TensorMem), C.c_int]
        lib.rknn_destroy_mem.argtypes = [Context, C.POINTER(TensorMem)]

        with open(model_path, "rb") as f:
            binary = f.read()
        self._model_data = C.create_string_buffer(binary, len(binary))
        self.ctx = Context()
        self.inputs, self.outputs = [], []
        start = time.perf_counter_ns()
        check(lib.rknn_init(C.byref(self.ctx), self._model_data, len(binary), 0, None), "rknn_init")
        self.init_ms = (time.perf_counter_ns() - start) / 1e6

        ver = SdkVersion()
        check(lib.rknn_query(self.ctx, RKNN_QUERY_SDK_VERSION, C.byref(ver), C.sizeof(ver)), "query version")
        self.api_version = ver.api_version.decode(errors="replace")
        self.drv_version = ver.drv_version.decode(errors="replace")

        io = IoNum()
        check(lib.rknn_query(self.ctx, RKNN_QUERY_IN_OUT_NUM, C.byref(io), C.sizeof(io)), "query io")
        self.inputs = [self._tensor(RKNN_QUERY_INPUT_ATTR, i) for i in range(io.n_input)]
        self.outputs = [self._tensor(RKNN_QUERY_OUTPUT_ATTR, i) for i in range(io.n_output)]

    def _tensor(self, query, index):
        attr = TensorAttr()
        attr.index = index
        check(self.lib.rknn_query(self.ctx, query, C.byref(attr), C.sizeof(attr)), "query attr")
        mem = self.lib.rknn_create_mem(self.ctx, attr.size_with_stride or attr.size)
        if not mem:
            raise RuntimeError("rknn_create_mem failed")
        if query == RKNN_QUERY_INPUT_ATTR:
            attr.pass_through = 1           # we write int16 already in the model's format
        check(self.lib.rknn_set_io_mem(self.ctx, mem, C.byref(attr)), "set_io_mem")
        return Tensor(attr, mem)

    def check_io(self):
        """Make sure the model is obs + h_in -> actions + h_out, all int16, and
        pick out each tensor by its size."""
        problems = []
        sizes_in = sorted(t.n for t in self.inputs)
        sizes_out = sorted(t.n for t in self.outputs)
        if sizes_in != sorted([N_OBS, N_HIDDEN]):
            problems.append("input sizes %s, expected %s" % (sizes_in, [N_OBS, N_HIDDEN]))
        if sizes_out != sorted([N_ACTIONS, N_HIDDEN]):
            problems.append("output sizes %s, expected %s" % (sizes_out, [N_ACTIONS, N_HIDDEN]))
        for t in self.inputs + self.outputs:
            if t.attr.type != RKNN_TENSOR_INT16:
                problems.append("%s is %s, expected int16" % (t.name, t.type_name))
        if problems:
            raise RuntimeError("model does not match this test:\n  " + "\n  ".join(problems))

        by_size_in = {t.n: t for t in self.inputs}
        by_size_out = {t.n: t for t in self.outputs}
        self.obs, self.h_in = by_size_in[N_OBS], by_size_in[N_HIDDEN]
        self.actions, self.h_out = by_size_out[N_ACTIONS], by_size_out[N_HIDDEN]
        # same quantization on h_out and h_in: feed back by a plain memory copy
        self.hidden_raw_copy = (self.h_in.scale, self.h_in.zp) == (self.h_out.scale, self.h_out.zp)

    def _sync_in(self, t):
        self.lib.rknn_mem_sync(self.ctx, t.mem, RKNN_MEMORY_SYNC_TO_DEVICE)

    def set_obs(self, raw):
        self.obs.write(raw)
        self._sync_in(self.obs)

    def set_hidden(self, raw):
        self.h_in.write(raw)
        self._sync_in(self.h_in)

    def feed_back_hidden(self):
        """h_in = h_out of the last run (call after get_outputs)."""
        if self.hidden_raw_copy:
            C.memmove(self.h_in.mem.contents.virt_addr, self.h_out.mem.contents.virt_addr,
                      N_HIDDEN * 2)
        else:
            self.h_in.write([self.h_in.to_raw(v) for v in self.h_out.to_real(self.h_out.read())])
        self._sync_in(self.h_in)

    def run(self):
        # synchronous: returns once the NPU has finished
        check(self.lib.rknn_run(self.ctx, None), "rknn_run")

    def get_outputs(self):
        """Returns the actions (raw int16); h_out stays in its buffer, read it
        with self.h_out.read() if needed."""
        for t in self.outputs:
            self.lib.rknn_mem_sync(self.ctx, t.mem, RKNN_MEMORY_SYNC_FROM_DEVICE)
        return self.actions.read()

    def close(self):
        if self.ctx.value:
            for t in self.inputs + self.outputs:
                self.lib.rknn_destroy_mem(self.ctx, t.mem)
            self.lib.rknn_destroy(self.ctx)
            self.ctx = Context()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

# =============================== test ===============================


def random_int16(n, low, high):
    return array.array("h", (random.randint(low, high) for _ in range(n)))


def show_values(label, tensor, raw, limit=None):
    raw = list(raw)
    more = " ... (+%d)" % (len(raw) - limit) if limit and len(raw) > limit else ""
    shown = raw[:limit] if limit else raw
    print("  %-8s raw : %s%s" % (label, shown, more))
    if tensor.attr.qnt_type != QNT_NONE:
        real = tensor.to_real(raw)
        print("  %-8s real: [%s]%s" % (label, ", ".join("%.4f" % v for v in real[:len(shown)]), more))
        if more:
            print("  %-8s real range %.4f .. %.4f" % (label, min(real), max(real)))


def print_stats(name, samples_ns):
    s = sorted(x / 1e6 for x in samples_ns)
    n = len(s)
    median = s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2
    p99 = s[min(n - 1, int(n * 0.99))]
    print("  %-30s mean %7.3f  median %7.3f  min %7.3f  p99 %7.3f  max %7.3f"
          % (name, sum(s) / n, median, s[0], p99, s[-1]))


def main():
    parser = argparse.ArgumentParser(
        description="Run the recurrent policy .rknn (obs[32] + h_in[512] -> actions[6] + h_out[512], "
                    "int16) on random obs and time it.")
    parser.add_argument("model", nargs="?", default=MODEL_PATH, help="default %(default)s")
    parser.add_argument("--runs", type=int, default=1000, help="timed runs (default %(default)s)")
    parser.add_argument("--warmup", type=int, default=20, help="untimed runs first (default %(default)s)")
    parser.add_argument("--samples", type=int, default=3, help="input -> output examples printed (default %(default)s)")
    parser.add_argument("--low", type=float, default=OBS_RANGE[0],
                        help="random obs min, real units (default %(default)s)")
    parser.add_argument("--high", type=float, default=OBS_RANGE[1],
                        help="random obs max, real units (default %(default)s)")
    parser.add_argument("--hidden", choices=("feedback", "zero", "random"), default="feedback",
                        help="h_in each run: h_out of the last run (like the real loop), "
                             "all 0, or random (default %(default)s)")
    parser.add_argument("--seed", type=int, help="random seed, for repeatable inputs")
    args = parser.parse_args()

    if args.low > args.high:
        parser.error("low must not be above high")
    if args.runs < 1:
        parser.error("runs must be at least 1")
    if args.seed is not None:
        random.seed(args.seed)
    if not os.path.isfile(args.model):
        parser.error("model not found: %s (set MODEL_PATH or pass the path)" % args.model)

    with RKNNModel(args.model) as model:
        print("lib   %s" % model.lib_path)
        print("rknn  api %s, NPU driver %s" % (model.api_version, model.drv_version))
        print("model %s, rknn_init %.1f ms" % (args.model, model.init_ms))
        for t in model.inputs:
            print("  in  " + t.describe())
        for t in model.outputs:
            print("  out " + t.describe())
        model.check_io()

        # random ranges in real units -> raw int16 for each tensor's scale
        obs_low, obs_high = model.obs.to_raw(args.low), model.obs.to_raw(args.high)
        h_low, h_high = model.h_in.to_raw(HIDDEN_RANGE[0]), model.h_in.to_raw(HIDDEN_RANGE[1])
        print("obs:    random %g .. %g (raw %d .. %d)" % (args.low, args.high, obs_low, obs_high))
        print("hidden: %s%s" % (args.hidden, " (h_out copied to h_in as raw int16)"
                                if args.hidden == "feedback" and model.hidden_raw_copy else ""))

        zero_hidden = array.array("h", [model.h_in.zp] * N_HIDDEN)   # raw for real 0

        def make_obs():
            return random_int16(N_OBS, obs_low, obs_high)

        def make_hidden():
            # h_in for one run, made before the clock starts; None = feed back
            if args.hidden == "zero":
                return zero_hidden
            if args.hidden == "random":
                return random_int16(N_HIDDEN, h_low, h_high)
            return None

        def step(obs, hidden):
            model.set_obs(obs)
            if hidden is None:
                model.feed_back_hidden()
            else:
                model.set_hidden(hidden)
            model.run()
            return model.get_outputs()

        # the hidden state starts at 0, like at the start of an episode
        model.set_hidden(zero_hidden)

        # the very first run is slower (NPU and caches warming up); shown, not in the stats
        obs = make_obs()
        t0 = time.perf_counter_ns()
        model.set_obs(obs)
        model.run()
        model.get_outputs()
        print("\nfirst run (cold): %.3f ms" % ((time.perf_counter_ns() - t0) / 1e6))

        # a few examples, each with new random obs
        for i in range(args.samples):
            obs, hidden = make_obs(), make_hidden()
            if hidden is None:
                h_in = model.h_out.read()                   # what feed_back_hidden will copy
            else:
                h_in = hidden
            actions = step(obs, hidden)
            print("\nsample %d:" % (i + 1))
            show_values("obs", model.obs, obs)
            show_values("h_in", model.h_in, h_in, limit=8)
            show_values("actions", model.actions, actions)
            show_values("h_out", model.h_out, model.h_out.read(), limit=8)

        for _ in range(args.warmup):
            step(make_obs(), make_hidden())

        # Timing: all random inputs are made before the loop, so only the model
        # is measured. perf_counter_ns is monotonic and integer (no float
        # rounding); gc is off so a garbage collection can't land in a sample.
        obs_list = [make_obs() for _ in range(args.runs)]
        hidden_list = [make_hidden() for _ in range(args.runs)]
        set_ns, run_ns, get_ns, total_ns = [], [], [], []
        clock = time.perf_counter_ns
        gc.disable()
        try:
            for obs, hidden in zip(obs_list, hidden_list):
                t0 = clock()
                model.set_obs(obs)
                if hidden is None:
                    model.feed_back_hidden()
                else:
                    model.set_hidden(hidden)
                t1 = clock()
                model.run()
                t2 = clock()
                model.get_outputs()
                t3 = clock()
                set_ns.append(t1 - t0)
                run_ns.append(t2 - t1)
                get_ns.append(t3 - t2)
                total_ns.append(t3 - t0)
        finally:
            gc.enable()

        resolution_us = time.get_clock_info("perf_counter").resolution * 1e6
        print("\ntime in ms over %d runs, each with new random obs, hidden %s "
              "(after %d warmup; clock resolution %.3f us):"
              % (args.runs, args.hidden, args.warmup, resolution_us))
        print_stats("set inputs (obs + h_in, sync)", set_ns)
        print_stats("rknn_run   (NPU inference)", run_ns)
        print_stats("get outputs (sync + read)", get_ns)
        print_stats("total", total_ns)


if __name__ == "__main__":
    main()
