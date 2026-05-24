"""Environment compatibility shims for the current py312 conda env.

Importing this module installs two import-time shims that allow `lerobot`
(commit 81948979) to load on the current `py312` env:

1.  A `sys.meta_path` finder that returns no-op stubs for any
    `flash_attn[.*]` import. After `pip uninstall flash_attn xformers` this
    is now mostly redundant, but kept defensively so that *anything* still
    referencing flash_attn (some HF model code does) won't blow up.

2.  A pre-stub for `lerobot.policies.groot.configuration_groot` so that
    `lerobot/policies/__init__.py` completes — needed because the upstream
    `groot_n1.py` uses `@strict` from `huggingface_hub.dataclasses` without
    first declaring `@dataclass`, which the installed huggingface_hub 1.7.1
    rejects with `StrictDataclassDefinitionError`.

Side effects happen at module import (top-level). Idempotent: importing
twice has no extra effect.

THIS IS A TEST-AND-DEV SHIM, NOT PRODUCTION CODE. Real fix is one line
upstream — add `@dataclass` before `@strict` on `GR00TN15Config`.
"""

from __future__ import annotations

import importlib.abc
import importlib.machinery
import sys
import types

_INSTALLED = False


def install() -> None:
    """Idempotent install of the two shims."""
    global _INSTALLED
    if _INSTALLED:
        return

    # --- 1. flash_attn no-op stub via meta-path finder ---
    class _FlashLoader(importlib.abc.Loader):
        def create_module(self, spec):
            m = types.ModuleType(spec.name)
            m.__spec__ = spec
            if spec.submodule_search_locations is not None:
                m.__path__ = []
            return m

        def exec_module(self, module):
            def _noop(*a, **k):
                return None

            if module.__name__ == "flash_attn":
                module.__version__ = "2.7.4"  # in xformers' accepted range
            for attr in (
                "flash_attn_func",
                "flash_attn_varlen_func",
                "flash_attn_with_kvcache",
                "flash_attn_gpu",
                "flash_attn_cuda",
                "index_first_axis",
                "pad_input",
                "unpad_input",
            ):
                setattr(module, attr, _noop)

    class _FlashFinder(importlib.abc.MetaPathFinder):
        def find_spec(self, fullname, path, target=None):
            if (
                fullname == "flash_attn"
                or fullname.startswith("flash_attn.")
                or fullname == "flash_attn_2_cuda"
            ):
                is_pkg = fullname == "flash_attn"
                return importlib.machinery.ModuleSpec(
                    fullname, _FlashLoader(), is_package=is_pkg
                )
            return None

    sys.meta_path.insert(0, _FlashFinder())

    # --- 2. Pre-stub broken groot config so lerobot.policies/__init__ completes ---
    groot_mod = "lerobot.policies.groot.configuration_groot"
    if groot_mod not in sys.modules:
        m = types.ModuleType(groot_mod)
        m.__spec__ = importlib.machinery.ModuleSpec(groot_mod, None)
        m.GrootConfig = type("GrootConfig", (), {})
        sys.modules[groot_mod] = m

    _INSTALLED = True


install()
