"""MapPickerDialog — shown when the operator selects NAVIGATE: pick which
saved map to localize against, or delete maps that are no longer wanted.

Delete calls back into the client (send_delete_map) immediately and drops the
row locally; the agent removes the files and re-broadcasts the map list, so the
next state keeps everything in sync. Deleting is confirmed first — the map
files are gone for good."""

from PySide6.QtWidgets import (QDialog, QDialogButtonBox, QHBoxLayout,
                               QListWidget, QMessageBox, QPushButton,
                               QVBoxLayout)


class MapPickerDialog(QDialog):
    def __init__(self, maps, delete_callback, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Navigate — choose a map")
        self.setMinimumWidth(280)
        self._delete_callback = delete_callback

        self._list = QListWidget()
        self._list.addItems(maps)
        self._list.setCurrentRow(0)
        self._list.itemDoubleClicked.connect(lambda _i: self.accept())

        self._del_btn = QPushButton("Delete")
        self._del_btn.clicked.connect(self._on_delete)

        self._buttons = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self._ok_btn = self._buttons.button(QDialogButtonBox.Ok)
        self._ok_btn.setText("Navigate")
        self._buttons.accepted.connect(self.accept)
        self._buttons.rejected.connect(self.reject)

        del_row = QHBoxLayout()
        del_row.addWidget(self._del_btn)
        del_row.addStretch(1)

        lay = QVBoxLayout(self)
        lay.addWidget(self._list, 1)
        lay.addLayout(del_row)
        lay.addWidget(self._buttons)
        self._sync_enabled()

    def _sync_enabled(self):
        has = self._list.count() > 0
        self._ok_btn.setEnabled(has)
        self._del_btn.setEnabled(has)

    def _on_delete(self):
        item = self._list.currentItem()
        if item is None:
            return
        name = item.text()
        if QMessageBox.question(
                self, "Delete map",
                f"Delete saved map '{name}'?\nThis permanently removes the map "
                "files and cannot be undone.") != QMessageBox.Yes:
            return
        self._delete_callback(name)
        self._list.takeItem(self._list.row(item))
        self._sync_enabled()

    def selected_map(self):
        item = self._list.currentItem()
        return item.text() if item else None
