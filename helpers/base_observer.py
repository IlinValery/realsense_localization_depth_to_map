from __future__ import annotations
from abc import ABC, abstractmethod


class BaseObserver(ABC):
    """
    The Observer interface declares the update method, used by subjects.
    """

    @abstractmethod
    def on_parent_update(self, subject: BaseSubject) -> None:
        """
        Receive update from subject.
        """
        pass


class BaseSubject(ABC):
    """
    The Subject interface declares a set of methods for managing subscribers.
    """

    @abstractmethod
    def attach(self, observer: BaseObserver) -> None:
        """
        Attach an observer to the subject.
        """
        pass

    @abstractmethod
    def detach(self, observer: BaseObserver) -> None:
        """
        Detach an observer from the subject.
        """
        pass

    @abstractmethod
    def notify(self) -> None:
        """
        Notify all observers about an event.
        """
        pass


