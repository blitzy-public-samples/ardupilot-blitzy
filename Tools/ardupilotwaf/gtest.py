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
    # The vendored GoogleTest (modules/gtest, release-1.8.0) predates the C++11 'override'
    # keyword and does not declare its internal helpers at file scope, so it trips the
    # -Werror=suggest-override / -Wmissing-declarations promotions that the standard board
    # CXXFLAGS carry (see Tools/ardupilotwaf/boards.py).  The suppressions are scoped to the
    # GoogleTest library only; modules/gtest itself is not modified.  This mirrors the existing
    # in-tree precedent for gSOAP (Tools/ardupilotwaf/ap_library.py) and for the gbenchmark
    # build (ap_find_benchmarks in Tools/ardupilotwaf/ardupilotwaf.py).
    # -Wno-maybe-uninitialized is additionally required for --debug (-O0) builds, where GCC's
    # analysis of testing::internal::StackGrowsDown() in gtest-death-test.cc flags the
    # deliberately-uninitialised probe variable.
    kw['cxxflags'] = Utils.to_list(kw.get('cxxflags', [])) + [
        '-Wno-undef',
        '-Wno-suggest-override',
        '-Wno-missing-declarations',
        '-Wno-maybe-uninitialized',
    ]
    kw.update(
        source='modules/gtest/googletest/src/gtest-all.cc',
        target='gtest/gtest',
        includes='modules/gtest/googletest modules/gtest/googletest/include',
        export_includes='modules/gtest/googletest/include',
        name='GTEST',
    )
    return bld.stlib(**kw)
