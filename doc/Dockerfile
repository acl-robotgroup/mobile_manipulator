# same python version as ros jazzy
FROM python:3.12-slim

ENV PYTHONDONTWRITEBYTECODE=1

COPY ./requirements.txt /requirements.txt

RUN pip3 install --no-cache-dir -r /requirements.txt

CMD ["sphinx-build", "-M", "html", "/src/doc", "/build"]
