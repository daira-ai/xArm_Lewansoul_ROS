%% Class is based on the default book class and options will be passed
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{daira-manual}[03-06-2022 v0.0.1 Daira AI Manual]

\DeclareOption*{\PassOptionsToClass{\CurrentOption}{book}}
\ProcessOptions\relax
\LoadClass[10pt,oneside]{book}

%% Main packages in the document --- Some are imported later in the class file
\RequirePackage{mathtools}  % Mathematical tools to use with amsmath
\RequirePackage{amssymb}    % Extended symbol collection
\RequirePackage{siunitx}    % Comprehensive (SI) units package

\RequirePackage{tabularx}   % Tabulars with adjustable-width columns
\RequirePackage{booktabs}   % Publication quality tables
\RequirePackage{longtable}  % Allow tables to flow over page boundaries
\RequirePackage{multirow}   % Create tabular cells spanning multiple rows

\RequirePackage{graphicx}   % Enhanced support for images
\RequirePackage{float}      % Improved interface for floating objects
\RequirePackage[labelfont=bf,justification=centering,footnotesize]{caption} % Captions
\RequirePackage{subcaption} % Support for sub-captions
\RequirePackage{pdfpages}   % Include PDF documents

\RequirePackage[pdfusetitle,hidelinks]{hyperref} % Extensive support for hypertext
\RequirePackage[noabbrev]{cleveref} % Intelligent cross-referencing
\RequirePackage{xcolor}     % Driver-independent color extensions
\RequirePackage{tikz}       % Create PostScript and PDF graphics
\RequirePackage{xspace}     % Define commands that appear not to eat spaces
\RequirePackage{microtype}  % Refinements towards typographical perfection

\RequirePackage{geometry}   % Customize document dimensions
\RequirePackage{titletoc}   % Alternative headings for toc
\RequirePackage{fancyhdr}   % Control of page headers and footers
\RequirePackage{enumitem}   % Control layout of itemize, enumerate, description
\RequirePackage{etoolbox}   % Toolbox of programming facilities
\RequirePackage{iftex}      % Adds if-else statements to support multiple compilers
\RequirePackage{datetime}   % Change format of \today

%% Defining commands for the cover image, subtitle, subject, copyright and table of authors
\newcommand*\subtitle[1]{\def\@subtitle{#1}}
\newcommand*\subject[1]{\def\@subject{#1}}
\newcommand*\dairacopyright[1]{\def\@dairacopyright{#1}}
\newcommand*\dairadateofmanual[1]{\def\@dairadateofmanual{#1}}
\newcommand*\dairarelease[1]{\def\@dairarelease{#1}}
\newcommand*\coverimage[1]{\def\@coverimage{#1}}
\newcommand*\covertable[1]{\def\@covertable{#1}}

%% Scaling the margins to be slightly smaller than default (.7)
\geometry{a4paper,hscale=0.75,vscale=0.8}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% I. Loading the fonts %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

\ifPDFTeX
    %% pdfLaTeX is available for compatibility and uses a Paletino-like font
    \RequirePackage[T1]{fontenc}  % Selecting T1 font encoding
    \RequirePackage{newpxtext}    % Palatino-like font...
    \RequirePackage{newpxmath}    % ...with support for mathematics

    %% Another font can be used as the regular and large title style font if desired
    \def\titlestyle{\normalfont}
    \def\largetitlestyle{\normalfont}

\else
    
    \DeclareRobustCommand{\robotostyle}{\fontseries{m}\selectfont}
    \DeclareRobustCommand{\largetitlestyle}{\fontseries{m}\selectfont}
    \DeclareRobustCommand{\titlestyle}{\fontseries{m}\selectfont}

    \DeclareRobustCommand{\ulseries}{\fontseries{ul}\selectfont}
    \DeclareRobustCommand{\lseries}{\fontseries{l}\selectfont}
    \DeclareRobustCommand{\mseries}{\fontseries{m}\selectfont}
    \DeclareRobustCommand{\mbseries}{\fontseries{mb}\selectfont}
    \DeclareRobustCommand{\bseries}{\fontseries{b}\selectfont}
    \DeclareRobustCommand{\ebseries}{\fontseries{eb}\selectfont}
    \DeclareTextFontCommand{\textul}{\ulseries}
    \DeclareTextFontCommand{\textl}{\lseries}
    \DeclareTextFontCommand{\textm}{\mseries}
    \DeclareTextFontCommand{\textmb}{\mbseries}
    \DeclareTextFontCommand{\textb}{\bseries}
    \DeclareTextFontCommand{\texteb}{\ebseries}


    %% Changing the quote environment to use Georgia
    %\AtBeginEnvironment{quote}{\quotefont}
\fi

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% II. Adjusting the layout of the titles %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Reducing the vertical white space between chapters in the table of contents
\dottedcontents{chapter}[1.5em]{\vspace{0.5\baselineskip}\bfseries}{1.5em}{0pt}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% III. Adjusting the header and footer %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Changing the header and footer of 'plain' pages
\fancypagestyle{plain}{%
    \fancyhf{}
    \renewcommand{\headrulewidth}{0pt}
    \fancyfoot[C]{\titlestyle\thepage}}

%% Changing the header and footer of 'fancy' pages (based on twoside option)
\if@twoside
    \fancyhf{}
    \fancyhead[LE,RO]{\titlestyle\thepage}
    \fancyhead[RE]{\titlestyle\nouppercase{\leftmark}}
    \fancyhead[LO]{\titlestyle\nouppercase{\rightmark}}
    \RequirePackage{emptypage} % Remove header and footer on empty pages
\else
    \fancyhf{}
    \fancyhead[R]{\titlestyle\thepage}
    \fancyhead[L]{\titlestyle\nouppercase{\rightmark}}
\fi

\pagestyle{fancy} % Setting the package defaults and the additional changes as the style

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% IV. Creating the cover page %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\newcolumntype{M}[1]{>{\centering\arraybackslash}m{#1}}
\newcommand*\makecover{
    \newdateformat{monthyeardate}{\monthname[\THEMONTH], \THEYEAR}
    %% Use the Tikz library positioning and clear the page header and footer
    \usetikzlibrary{positioning}
    \thispagestyle{empty}

    %% Constructing the cover page (cover image, name of university, logo and title text respectively)
    \begin{tikzpicture}[overlay,remember picture]

    \node[above=0,inner sep=0] at (current page.south) {\includegraphics[width=\paperwidth]{\@coverimage}};

    \node[scale=.6,below left=239mm and 10mm] at (current page.north east) {%
        \includegraphics[width=0.50\linewidth]{Daira_Logo.png}};
    
    \node[above,fill=white!85,minimum width={9cm},right=15mm, yshift=30mm, inner ysep=20pt, opacity=0.6,text opacity=1] at (current page.south west) {       
        \begin{minipage}{8cm}
            \vspace*{0mm}
            \color{black}\fontsize{14}{14}\selectfont%
            \textl{Date of issue: \monthyeardate\today}
            \vspace*{4mm}


            \textl{Release Version: \@dairarelease}
        \end{minipage}
    };

    \end{tikzpicture}
    \newpage

    \begin{tikzpicture}[overlay,remember picture]
        \node[xshift=0mm,yshift=20mm,anchor=south] at (current page.south){% 
        \parbox{\textwidth}{%
            
            \vspace*{3mm}
            \textbf{Disclaimer:} \textl{The information in this user manual is provided by Daira AI.
            The information is intended for the use of the customers and clients of Daira AI only. No part of this user manual may be reproduced,
            distributed, or transmitted in any form or by any means, without the prior written permission of Daira AI. Daira AI reserves the right
            to make changes to the information in this user manual at any time without notice. Daira AI is not liable for any errors or omissions
            in this user manual, or for any damages arising from the use of this user manual.}
            \vspace*{3mm}
            % \textul{Abc Thin} {\ulseries Thin}
            % \textl{Abc Light} {\lseries Light}
            % \textm{Abc Medium} {\mseries Medium}
            % \textb{Abc Bold} {\bseries Bold}
            % \texteb{Abc ExtraBold} {\ebseries ExtraBold}
        }%
    };
    \end{tikzpicture}

    \newpage
}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% V. Setting up listings %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

\RequirePackage{listings} % Typeset source code listings

\lstset{
  basicstyle=\ttfamily\footnotesize,  % Style of the font that is used for the code
  backgroundcolor=\color{gray!10},    % Background color
  keywordstyle=\color{red!75!black},  % Keyword style
  stringstyle=\color{green!40!black}, % String style
  commentstyle=\color{blue!30!black}, % Comment style
  numbers=left,                       % Add line numbers on the left side
  numbersep=5pt,                      % Decrease distance between line numbers and code
  numberstyle=\tiny,                  % Line number style
  breaklines=true,                    % Line break automatically
}
