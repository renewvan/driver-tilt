import pytest

from driver_tilt import publisher as publisher_module
from driver_tilt.config import MqttConfig
from driver_tilt.publisher import HEALTH_TOPIC, INFO_TOPIC, Publisher

MQTT = MqttConfig(
    host="localhost",
    port=1883,
    username=None,
    password=None,
    client_id="test-tilt",
    reconnect_min_delay=1,
    reconnect_max_delay=2,
)

INFO = '{"device": "tilt", "version": "dev"}'


class FakeClient:
    """Stands in for paho's Client. Records publishes in order and keeps the
    callbacks the Publisher registers, so a test can play the broker's part."""

    def __init__(self, *args, **kwargs) -> None:
        self.published: list[tuple[str, str, bool]] = []
        self.will: tuple[str, str, bool] | None = None
        self.on_connect = None

    def username_pw_set(self, *args) -> None: ...

    def will_set(self, topic, payload, qos, retain) -> None:
        self.will = (topic, payload, retain)

    def reconnect_delay_set(self, **kwargs) -> None: ...

    def publish(self, topic, payload=None, qos=0, retain=False) -> None:
        self.published.append((topic, payload, retain))

    def subscribe(self, *args, **kwargs) -> None: ...

    def connect(self, *args) -> None: ...

    def loop_start(self) -> None: ...

    def loop_stop(self) -> None: ...

    def disconnect(self) -> None: ...

    def broker_accepts_connection(self, rc: int = 0) -> None:
        """What paho does after CONNACK, on first connect and on every reconnect."""
        self.on_connect(self, None, {}, rc)


@pytest.fixture
def client(monkeypatch) -> FakeClient:
    fake = FakeClient()
    monkeypatch.setattr(publisher_module.mqtt, "Client", lambda *a, **k: fake)
    Publisher(MQTT, INFO).connect()
    return fake


def test_info_is_published_retained_immediately_before_online(client) -> None:
    client.broker_accepts_connection()
    assert client.published == [(INFO_TOPIC, INFO, True), (HEALTH_TOPIC, "online", True)]


def test_info_is_published_again_on_every_reconnect(client) -> None:
    client.broker_accepts_connection()
    client.broker_accepts_connection()
    assert client.published == [
        (INFO_TOPIC, INFO, True),
        (HEALTH_TOPIC, "online", True),
        (INFO_TOPIC, INFO, True),
        (HEALTH_TOPIC, "online", True),
    ]


def test_failed_connect_publishes_nothing(client) -> None:
    client.broker_accepts_connection(rc=5)
    assert client.published == []


def test_info_stays_retained_when_the_device_goes_offline(monkeypatch) -> None:
    fake = FakeClient()
    monkeypatch.setattr(publisher_module.mqtt, "Client", lambda *a, **k: fake)
    publisher = Publisher(MQTT, INFO)
    publisher.connect()
    fake.broker_accepts_connection()
    fake.published.clear()

    publisher.disconnect()

    # A clean shutdown only flips health; nothing is published to the info topic,
    # so the broker keeps the retained info.
    assert fake.published == [(HEALTH_TOPIC, "offline", True)]
    # The last will (a crash) likewise only flips health.
    assert fake.will == (HEALTH_TOPIC, "offline", True)
