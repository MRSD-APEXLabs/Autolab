FROM python:3.12-alpine
WORKDIR /app
COPY mix_colors.py pair_test_results.json server.py ./
EXPOSE 8090
CMD ["python3", "-u", "server.py"]
