# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html


# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinxcontrib.matlab', 
    'sphinx.ext.autodoc',
    "sphinx.ext.autosummary",
    'sphinx.ext.duration',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    "sphinxcontrib.katex",
    "sphinx.ext.autosectionlabel",
    "sphinx_copybutton",
    "myst_parser",
    "sphinx_design",
]
myst_enable_extensions = [
    "colon_fence",
    "deflist",
    "substitution",
    "html_image",
]

sd_fontawesome_latex = False

katex_prerender = True

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']
autosummary_generate = True  # Turn on sphinx.ext.autosummary

# -- Napoleon Docstrings Configuration --------------------------------------

napoleon_use_param = False
napoleon_use_keyword = False

# -- Additional options for sphinxcontrib.matlab ----------------------------
# https://pypi.org/project/sphinxcontrib-matlabdomain/
import os

pwd = os.path.dirname(os.path.abspath(__file__))
matlab_src_dir = os.path.abspath(os.path.join(pwd, '../..'))
matlab_keep_package_prefix = False
#primary_domain='mat'
autodoc_member_order='groupwise'

# -- Options for HTML output ------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
#html_static_path = ['_static']
html_theme_options = {
    'collapse_navigation': False,
    'sticky_navigation': True
}

# -- Special Ignore Configuration -------------------------------------------

from sphinx.ext.autodoc import between

def setup(app):
    # Register a sphinx.ext.autodoc.between listener to ignore everything
    # after lines that contain the word IGNORE
    app.connect('autodoc-process-docstring', between('^.*--- More Info ---.*$', exclude=True))
    return app

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

VERSION = os.environ.get("VERSION", 'dev')

if VERSION == 'dev':
    project = 'RTD-code MATLAB (dev)'
    version = 'development version'
    version_match = 'dev'
else:
    project = 'RTD-code MATLAB'
    version = VERSION
    version_match = VERSION

copyright = '2024, ROAHM Lab'
author = 'ROAHM Lab'
release = version
html_title = f'{project} {version}'

# TODO Add version selector

