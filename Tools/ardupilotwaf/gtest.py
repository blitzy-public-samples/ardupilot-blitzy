# encoding: utf-8

# flake8: noqa

"""
gtest is a Waf tool for test builds in Ardupilot
"""

from waflib import Utils
from waflib.Configure import conf

import boards

def configure(cfg):
    cfg.env.HAS_GTEST = False
    if cfg.options.disable_tests:
        return

    board = cfg.get_board()
    if isinstance(board, boards.chibios):
        # toolchain is currently broken for gtest
        cfg.msg(
            'Gtest',
            'STM32 boards currently don\'t support compiling gtest',
            color='YELLOW',
        )
        return

    if cfg.env.STATIC_LINKING:
        # gtest uses a function (getaddrinfo) that is supposed to be linked
        # dynamically
        cfg.msg(
            'Gtest',
            'statically linked tests not supported',
            color='YELLOW',
        )
        return

    cfg.env.append_value('GIT_SUBMODULES', 'gtest')
    cfg.env.HAS_GTEST = True

@conf
def libgtest(bld, **kw):
    # CODE ISSUE #2: the vendored googletest (modules/gtest, ~1.8.0) predates the
    # 'override' specifier and emits -Werror=suggest-override, and gtest-all.cc
    # also trips -Werror=missing-declarations. The SITL board enables both as
    # errors unconditionally (Tools/ardupilotwaf/boards.py: sitl.configure_env
    # appends -Werror=missing-declarations; the base GCC path appends
    # -Werror=suggest-override for cc >= 5.2), independent of --debug. In addition,
    # under a --debug build (-O0, used by run_coverage.py / the test_coverage CI
    # workflow which reconfigures --board=... --debug --coverage), GCC 15's
    # data-flow analysis flags gtest-death-test.cc:1009 with
    # -Werror=maybe-uninitialized on the vendored 'dummy' local. All of these
    # suppressions are therefore required for the GTEST static library to compile
    # on this toolchain across both the default and --debug configurations. The
    # scope is limited to this libgtest stlib so no other warnings are masked, and
    # the submodule is consumed as-is, never edited (AAP sections 0.5.3 and 0.10).
    kw['cxxflags'] = Utils.to_list(kw.get('cxxflags', [])) + ['-Wno-undef', '-Wno-suggest-override', '-Wno-missing-declarations', '-Wno-maybe-uninitialized']
    kw.update(
        source='modules/gtest/googletest/src/gtest-all.cc',
        target='gtest/gtest',
        includes='modules/gtest/googletest modules/gtest/googletest/include',
        export_includes='modules/gtest/googletest/include',
        name='GTEST',
    )
    return bld.stlib(**kw)
