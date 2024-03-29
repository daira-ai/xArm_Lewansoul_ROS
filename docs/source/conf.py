# -*- coding: utf-8 -*-
#
# Configuration file for the Sphinx documentation builder.
#
# This file does only contain a selection of the most common options. For a
# full list see the documentation:
# http://www.sphinx-doc.org/en/master/config

from datetime import date
import os
from pygit2 import Repository

# -- Sphinx and Markdown configuration  -----------------------------------------------------
import recommonmark
from recommonmark.parser import CommonMarkParser
from recommonmark.transform import AutoStructify

source_parsers = {
    '.md': CommonMarkParser,
}

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
# import sys
# sys.path.insert(0, os.path.abspath('.'))


# -- Project information -----------------------------------------------------

project = u'xArm Lewansoul ROS'
current_year = str(date.today().year)
copyright = u'2023-'+str(current_year)+', Daira AI'
author = 'Daira AI'

github_doc_root = 'https://https://github.com/daira-ai/xArm_Lewansoul_ROS/blob/melodic-devel/docs/index.rst'

# The short X.Y version
current_path = os.path.abspath('.')
current_branch = Repository('.').head.shorthand
version = "\detokenize{"+current_branch+"}"
# The full version, including alpha/beta/rc tags
release = current_branch


# -- General configuration ---------------------------------------------------

# If your documentation needs a minimal Sphinx version, state it here.
#
# needs_sphinx = '1.0'

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.doctest',
    'sphinx.ext.intersphinx',
    'sphinx.ext.viewcode',
    'sphinx.ext.githubpages',
    'sphinx-prompt',
    'notfound.extension',
]

# Add any paths that contain templates here, relative to this directory.
# templates_path = ['_templates']

# The suffix(es) of source filenames.
# You can specify multiple suffix as a list of string:
#
source_suffix = ['.rst', '.md']

# The master toctree document.
master_doc = 'index'

# The language for content autogenerated by Sphinx. Refer to documentation
# for a list of supported languages.
#
# This is also used if you do content translation via gettext catalogs.
# Usually you set "language" from the command line for these cases.
language = 'en'

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path .
exclude_patterns = [u'_build', 'Thumbs.db', '.DS_Store']

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = 'sphinx'


# -- Options for HTML output -------------------------------------------------

html_theme = 'sphinx_rtd_theme'

# Theme options are theme-specific and customize the look and feel of a theme
# further.  For a list of options available for each theme, see the
# documentation.

html_theme_options = {
    'style_nav_header_background': '#fcfcfc',
    'logo_only': True,
    'display_version': False,
}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

# The name of an image file (within the static path) to use as favicon of the
# docs.  This file should be a Windows icon file (.ico) being 16x16 or 32x32
# pixels large.
html_favicon = '_static/logos/faviconDaira.png'

html_logo = "_static/logos/Daira_Logo.png"


# -- Options for HTMLHelp output ---------------------------------------------

# Output file base name for HTML help builder.
htmlhelp_basename = 'xArm_Lewansoul_ROS_doc'


# -- Options for LaTeX output ------------------------------------------------

latex_elements = {
    # The paper size ('letterpaper' or 'a4paper').
    #
    'papersize': 'letterpaper',

    # The font size ('10pt', '11pt' or '12pt').
    #
    'pointsize': '10pt',

    # Latex figure (float) alignment
    # 'figure_align': 'htbp',
    'fontpkg': r'''
    \usepackage{fontspec}
    \setsansfont{Roboto}[
        Path = ,
        Extension = .ttf,
        UprightFont = *-Regular,
        %-- Upright --%
        FontFace={ul}{n}{Font=*-Thin},
        FontFace={l}{n}{Font=*-Light},
        FontFace={m}{n}{Font=*-Regular},
        FontFace={mb}{n}{Font=*-Medium},
        FontFace={b}{n}{Font=*-Bold},
        FontFace={eb}{n}{Font=*-Black},
        % %-- Italic --%
        FontFace={ul}{it}{Font=*-ThinItalic},
        FontFace={l}{it}{Font=*-LightItalic},
        FontFace={m}{it}{Font=*-Italic},
        FontFace={mb}{it}{Font=*-MediumItalic},
        FontFace={b}{it}{Font=*-BoldItalic},
        FontFace={eb}{it}{Font=*-BlackItalic},
    ]
    \setmainfont{Roboto}[
        Path = ,
        Extension = .ttf,
        UprightFont = *-Light,
        %-- Upright --%
        FontFace={ul}{n}{Font=*-Thin},
        FontFace={l}{n}{Font=*-Light},
        FontFace={m}{n}{Font=*-Regular},
        FontFace={mb}{n}{Font=*-Medium},
        FontFace={b}{n}{Font=*-Bold},
        FontFace={eb}{n}{Font=*-Black},
        % %-- Italic --%
        FontFace={ul}{it}{Font=*-ThinItalic},
        FontFace={l}{it}{Font=*-LightItalic},
        FontFace={m}{it}{Font=*-Italic},
        FontFace={mb}{it}{Font=*-MediumItalic},
        FontFace={b}{it}{Font=*-BoldItalic},
        FontFace={eb}{it}{Font=*-BlackItalic},
    ]
    ''',

    # Additional stuff for the LaTeX preamble.
    'preamble': r'''
    \titleformat{\chapter}[display]
        {\flushright}
        {\fontsize{96}{96}\selectfont\largetitlestyle\thechapter}
        {0pt}
        {\Huge\titlestyle}
    \titlespacing*{\chapter}{0pt}{0pt}{2\baselineskip}

    %% Formatting section titles and spacing
    \titleformat{\section}
        {\Large\titlestyle}
        {\thesection.}
        {5pt}
        {}
    \titlespacing*{\section}{0pt}{\baselineskip}{0pt}

    %% Formatting subsections titles and spacing
    \titleformat{\subsection}
        {\large\titlestyle}
        {\thesubsection.}
        {5pt}
        {}
    \titlespacing*{\subsection}{0pt}{\baselineskip}{0pt}

    %% Formatting subsubsections titles and spacing
    \titleformat{\subsubsection}
        {\titlestyle}
        {}
        {0pt}
        {}
    \titlespacing*{\subsubsection}{0pt}{\bigskipamount}{0pt}
    ''',

    'maketitle': r'''
    \pagenumbering{Roman}
    \begin{titlepage}

    %% Defining the main parameters
    
    %\author{Daira AI}
    %\subject{Manual}
    \dairacopyright{Copyright © 2023-''' + current_year + r''' by Daira AI. All rights reserved}
    \dairarelease{''' + "\detokenize{"+current_branch+"}" + r'''}

    \coverimage{ManualCover.png}
    \definecolor{title}{HTML}{D00070} % Color for title
    \makecover

    \end{titlepage}
    \clearpage
    \tableofcontents
    \clearpage
    \pagenumbering{arabic}
    ''',

    'sphinxsetup':'hmargin={0.7in,0.7in}, vmargin={1in,1in}',

    'tableofcontents':' ',
}

latex_docclass = {
   'manual': 'daira-manual',
}
latex_logo = '_static/logos/Daira_Logo.png'
latex_engine = 'xelatex'
latex_theme_path = ['_static']
latex_additional_files = ['_static/latex-layout/daira-manual.cls', '_static/latex-layout/ManualCover.png',
                          '_static/logo/Daira_Logo.png',
                          '_static/latex-layout/fonts/Roboto/Roboto-Black.ttf',
                          '_static/latex-layout/fonts/Roboto/Roboto-BlackItalic.ttf',
                          '_static/latex-layout/fonts/Roboto/Roboto-Bold.ttf',
                          '_static/latex-layout/fonts/Roboto/Roboto-BoldItalic.ttf',
                          '_static/latex-layout/fonts/Roboto/Roboto-Italic.ttf',
                          '_static/latex-layout/fonts/Roboto/Roboto-Light.ttf',
                          '_static/latex-layout/fonts/Roboto/Roboto-LightItalic.ttf',
                          '_static/latex-layout/fonts/Roboto/Roboto-Medium.ttf',
                          '_static/latex-layout/fonts/Roboto/Roboto-MediumItalic.ttf',
                          '_static/latex-layout/fonts/Roboto/Roboto-Regular.ttf',
                          '_static/latex-layout/fonts/Roboto/Roboto-Thin.ttf',
                          '_static/latex-layout/fonts/Roboto/Roboto-ThinItalic.ttf',
                          ]

# Grouping the document tree into LaTeX files. List of tuples
# (source start file, target name, title,
#  author, documentclass [howto, manual, or own class]).
latex_documents = [
    (master_doc, 'xArm_Lewansoul_ROS.tex', u'xArm Lewansoul ROS Documentation',
     u'Daira AI', 'manual'),
]


# -- Options for manual page output ------------------------------------------

# One entry per manual page. List of tuples
# (source start file, name, description, authors, manual section).
man_pages = [
    (master_doc, 'xArm_Lewansoul_ROS', u'xArm Lewansoul ROS Documentation',
     [author], 1)
]


# -- Options for Texinfo output ----------------------------------------------

# Grouping the document tree into Texinfo files. List of tuples
# (source start file, target name, title, author,
#  dir menu entry, description, category)
texinfo_documents = [
    (master_doc, 'xArm_Lewansoul_ROS', u'xArm Lewansoul ROS Documentation',
     author, 'xArm_Lewansoul_ROS', 'One line description of project.',
     'Miscellaneous'),
]


# -- Options for Epub output -------------------------------------------------

# Bibliographic Dublin Core info.
epub_title = project
epub_author = author
epub_publisher = author
epub_copyright = copyright

# The unique identifier of the text. This can be a ISBN number
# or the project homepage.
#
# epub_identifier = ''

# A unique identification for the text.
#
# epub_uid = ''

# A list of files that should not be packed into the epub file.
epub_exclude_files = ['search.html']


# -- Extension configuration -------------------------------------------------

# -- Options for intersphinx extension ---------------------------------------

# Example configuration for intersphinx: refer to the Python standard library.
intersphinx_mapping = {'python': ('https://docs.python.org/3', None)}

# sphinx-notfound-page
# https://github.com/rtfd/sphinx-notfound-page
# notfound_context = {
#     'title': 'Page not found',
#     'body': "<h1>Page not found</h1>\n\nUnfortunately we couldn't find the content you were looking for.",
# }

# -- Options for Markdown output -------------------------------------------------
# app setup hook
def setup(app):
    app.add_css_file('css/daira_docs_style.css')
    app.add_config_value('recommonmark_config', {
        'auto_toc_tree_section': 'Contents',
        'enable_eval_rst': True,
    }, True)
