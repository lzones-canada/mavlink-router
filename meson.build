project(
        'mavlink-router',
        'cpp', 'c',
        version: '3',
        license: 'Apache 2.0',
        default_options: [
                'cpp_std=gnu++14',
                'prefix=/usr',
                'warning_level=2',
                'buildtype=debugoptimized',
        ],
        meson_version : '>= 0.55',
)

project_source_root = meson.current_source_dir()

conf = configuration_data()
conf.set('BINDIR', join_paths(get_option('prefix'), get_option('bindir')))
conf.set_quoted('VERSION', meson.project_version(), description : 'Project version')
conf.set_quoted('PACKAGE', meson.project_name())

cxx = meson.get_compiler('cpp')
cc = meson.get_compiler('c')

# Mandatory dependencies
dep_math = cxx.find_library('m')
dep_rt = cxx.find_library('rt')
dep_thread = dependency('threads')

# Optional dependencies
systemd_system_unit_dir = get_option('systemdsystemunitdir')
if systemd_system_unit_dir == 'auto'
		dep_systemd = dependency('systemd')
		systemd_system_unit_dir = dep_systemd.get_variable(pkgconfig: 'systemdsystemunitdir')
endif

dep_gtest = dependency('gtest', main : true, required : false)

# Additional warnings
possible_cc_flags = [
	    '-Wno-inline',
	    '-Wundef',
	    '-Wformat=2',
	    '-Wlogical-op',
	    '-Wsign-compare',
	    '-Wformat-security',
	    '-Wmissing-include-dirs',
	    '-Wformat-nonliteral',
	    '-Wold-style-definition',
	    '-Wpointer-arith',
	    '-Winit-self',
	    '-Wdeclaration-after-statement',
	    '-Wfloat-equal',
	    '-Wmissing-prototypes',
	    '-Wstrict-prototypes',
	    '-Wredundant-decls',
	    '-Wmissing-declarations',
	    '-Wmissing-noreturn',
	    '-Wshadow',
	    '-Wendif-labels',
	    '-Wstrict-aliasing=3',
	    '-Wwrite-strings',
	    '-Wno-long-long',
	    '-Wno-overlength-strings',
	    '-Wno-unused-parameter',
	    '-Wno-missing-field-initializers',
	    '-Wno-unused-result',
	    '-Wnested-externs',
	    '-Wchar-subscripts',
	    '-Wtype-limits',
	    '-Wuninitialized',
]

possible_cxx_flags = [
        '-Waddress-of-packed-member',
        '-Wno-inline',
        '-Wundef',
        '-Wformat=2',
        '-Wlogical-op',
        '-Wsign-compare',
        '-Wformat-security',
        '-Wmissing-include-dirs',
        '-Wformat-nonliteral',
        '-Wpointer-arith',
        '-Winit-self',
        '-Wfloat-equal',
        '-Wredundant-decls',
        '-Wmissing-declarations',
        '-Wmissing-noreturn',
        '-Wshadow',
        '-Wendif-labels',
        '-Wstrict-aliasing=3',
        '-Wwrite-strings',
        '-Wno-long-long',
        '-Wno-overlength-strings',
        '-Wno-unused-parameter',
        '-Wno-missing-field-initializers',
        '-Wno-unused-result',
        '-Wchar-subscripts',
        '-Wtype-limits',
        '-Wuninitialized',
]

add_project_arguments(cc.get_supported_arguments(possible_cc_flags), language : 'c')
add_project_arguments(cc.get_supported_arguments(possible_cxx_flags), language : 'cpp')

# System or compiler dependent tweaks
has_waddress_of_packed_member = cxx.has_argument('-Waddress-of-packed-member')
conf.set10('HAVE_WADDRESS_OF_PACKED_MEMBER', has_waddress_of_packed_member)

has_aio_init_symbols = cxx.has_header_symbol('aio.h', 'aio_init')
conf.set10('HAVE_DECL_AIO_INIT', has_aio_init_symbols)

# Always include config.h
config_h = configure_file(output : 'config.h', configuration : conf)
add_project_arguments('-include', 'config.h', language : 'cpp')
add_project_arguments('-include', 'config.h', language : 'c')

# Git version tag
vcs_tagger = [project_source_root + '/tools/meson-vcs-tag.sh',
              project_source_root]
version_h = vcs_tag(input : 'src/version.h.in',
                    output: 'git_version.h',
                    command: vcs_tagger)

# MAVLink C headers are generated at setup time from the mavlink submodule
# (the lzones-canada fork, pinned to the commit ArduPilot uses), so the
# message table always matches the firmware's dialect. Setup re-runs, and
# regenerates them, when any of the dialect's message definitions change.
python3 = find_program('python3')
mavlink_dir = project_source_root / 'modules/mavlink'
fs = import('fs')
if not (fs.is_file(mavlink_dir / 'message_definitions/v1.0/ardupilotmega.xml') and
        fs.is_file(mavlink_dir / 'pymavlink/generator/mavgen.py'))
        error('modules/mavlink (with its pymavlink) is missing or not initialised; ' +
              'run "git submodule sync --recursive && git submodule update --init --recursive"')
endif
mavlink_gen = run_command(python3, project_source_root / 'tools/mavlink-gen.py',
                          mavlink_dir, meson.current_build_dir() / 'mavlink',
                          check: false)
if mavlink_gen.returncode() != 0
        error('MAVLink header generation failed:\n' + mavlink_gen.stderr())
endif
foreach xml : mavlink_gen.stdout().strip().split('\n')
        configure_file(input: mavlink_dir / 'message_definitions/v1.0' / xml,
                       output: 'mavlink-dep-' + xml,
                       copy: true)
endforeach

# The generated headers only exist in the build dir, so add them as a plain
# include path: include_directories() would also add a non-existent source
# path and trip -Wmissing-include-dirs.
add_project_arguments('-I' + meson.current_build_dir() / 'mavlink/ardupilotmega',
                      language: ['c', 'cpp'])

# Includes
mavlink_inc = include_directories()
src_inc = include_directories('src')

# Additonal data to be installed
configure_file(
        input: 'mavlink-router.service.in',
        output: 'mavlink-router.service',
        configuration: conf,
        install_dir: systemd_system_unit_dir,
)

subdir('src')
subdir('examples')

# Use clang-tidy to modernize code and check for common code style issues
clangtidy = find_program('run-clang-tidy', required: false)
clangtidy_wrapper = files('tools/meson-clang-tidy.sh')
if clangtidy.found()
        # common set of auto-fixable issues
        run_target('clangtidy-autofix',
                command : [clangtidy_wrapper, '-p', meson.current_build_dir(),
                        '-header-filter', '@SOURCE_ROOT@/src/.*',
                        '-fix', '-format',
                        ])
        # check, which need manual evaluation on a case by case basis
        run_target('clangtidy-check',
                command : [clangtidy_wrapper, '-p', meson.current_build_dir(),
                        '-header-filter', '@SOURCE_ROOT@/src/.*',
                        '-checks=bugprone-*,-bugprone-easily-swappable-parameters,cppcoreguidelines-*,-cppcoreguidelines-avoid-goto,-cppcoreguidelines-macro-usage,-cppcoreguidelines-owning-memory,-cppcoreguidelines-pro-bounds-constant-array-index,-cppcoreguidelines-pro-bounds-pointer-arithmetic,-cppcoreguidelines-pro-type-union-access,-cppcoreguidelines-pro-type-vararg,misc-*,-misc-misleading-identifier,-misc-no-recursion,modernize-concat-nested-namespaces,modernize-replace-auto-ptr,modernize-replace-disallow-copy-and-assign-macro,modernize-replace-random-shuffle,modernize-shrink-to-fit,modernize-unary-static-assert,modernize-use-transparent-functors,modernize-use-uncaught-exceptions,performance-*,readability-avoid-const-params-in-decls,readability-container-size-empty,readability-convert-member-functions-to-static,readability-misleading-indentation,readability-misplaced-array-index,readability-named-parameter,readability-non-const-parameter,readability-redundant-access-specifiers,readability-redundant-control-flow,readability-redundant-preprocessor,readability-simplify-subscript-expr,readability-static-accessed-through-instance,readability-static-definition-in-anonymous-namespace,readability-suspicious-call-argument,readability-uppercase-literal-suffix',
                        ])
endif
