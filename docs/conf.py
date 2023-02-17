# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Inverted Pendulum'
copyright = '2023, ICraveSleep'
author = 'ICraveSleep'
release = 'v0.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

# Math support> https://sphinx-rtd-trial.readthedocs.io/en/latest/ext/math.html#module-sphinx.ext.mathbase
# Read the docs theme: https://sphinx-rtd-theme.readthedocs.io/en/stable/index.html
# Requires to install rtd theme: pip install sphinx_rtd_theme
extensions = ['sphinx_rtd_theme',
              'sphinx.ext.mathjax']

# 'sphinx.ext.imgmath'

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

# Customs
# CSS custom: https://stackoverflow.com/questions/62626722/removing-borders-from-a-table-in-sphinx-documentation
html_css_files = [
    'css/custom.css',
]
