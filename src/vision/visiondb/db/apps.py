from django.apps import AppConfig


class DbConfig(AppConfig):
    name = 'db'

    def ready(self):
        import db.signals
