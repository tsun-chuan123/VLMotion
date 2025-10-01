"""
Lightweight package initializer for `point`.

Avoid importing heavy model classes at import time to ensure services
like the controller can start without ML dependencies present. Modules
that need models should import from `point.model` explicitly.
"""

__all__ = []
