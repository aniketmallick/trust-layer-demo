def pytest_configure(config):
    config.addinivalue_line("markers", "live: needs the operator's .env and a saved frame; calls the provider")
