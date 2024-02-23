FROM sphinxdoc/sphinx:7.1.2

ENV DEBIAN_FROTNEND="noninteractive" TZ="Etc/UTC"
RUN apt-get update \
    && apt-get install --no-install-recommends -y nodejs texlive-latex-recommended texlive-latex-extra texlive-fonts-recommended tex-gyre latexmk \
    && apt-get clean \
    && apt-get -y autoremove \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt && rm /tmp/requirements.txt
