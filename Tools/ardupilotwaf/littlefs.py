# encoding: utf-8

# flake8: noqa

"""
Adds support for building littlefs as part of a Waf build
"""

from waflib.Configure import conf

def configure(cfg):
    cfg.env.append_value('GIT_SUBMODULES', 'littlefs')
    cfg.env.prepend_value('INCLUDES', [
        cfg.srcnode.abspath() + '/modules/littlefs/',
    ])


@conf
def littlefs(bld, **kw):
    kw.update(
        name='littlefs',
        source=['modules/littlefs/lfs.c', 'modules/littlefs/lfs_util.c', 'modules/littlefs/bd/lfs_filebd.c'],
        target='littlefs',
        defines=['LFS_NO_DEBUG', 'LFS_NO_WARN', 'LFS_NO_ERROR', 'LFS_NO_ASSERT'],
        # -Wno-unused-variable: littlefs is compiled with LFS_NO_ASSERT (see defines above), which
        # expands LFS_ASSERT() to nothing.  In lfs_filebd_erase() the only consumer of the local
        # 'bd' is that assertion, so with assertions disabled the variable becomes unused and trips
        # the -Werror=unused-variable promotion carried by the standard board CXXFLAGS/CFLAGS.
        # The suppression is scoped to this vendored library only; modules/littlefs is not modified.
        cflags=['-Wno-format-security', '-Wno-format', '-Wno-format-extra-args', '-Wno-shadow', '-Wno-unused-function', '-Wno-missing-declarations', '-Wno-unused-variable']
    )
    return bld.stlib(**kw)
