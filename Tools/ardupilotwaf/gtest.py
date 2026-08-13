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
    kw['cxxflags'] = Utils.to_list(kw.get('cxxflags', [])) + ['-Wno-undef', '-Wno-suggest-override', '-Wno-missing-declarations']
    # gcc 15 flags the deliberately-uninitialised stack probe in the vendored gtest-death-test.cc
    # at -O0; demote the promotion for this library so the diagnostic stays reported but non-fatal
    if '-Werror=maybe-uninitialized' in bld.env.CXXFLAGS:
        kw['cxxflags'] += ['-Wno-error=maybe-uninitialized']
    kw.update(
        source='modules/gtest/googletest/src/gtest-all.cc',
        target='gtest/gtest',
        includes='modules/gtest/googletest modules/gtest/googletest/include',
        export_includes='modules/gtest/googletest/include',
        name='GTEST',
    )
    return bld.stlib(**kw)
