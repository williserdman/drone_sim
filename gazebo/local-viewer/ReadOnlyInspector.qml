import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3

Rectangle {
  color: Material.theme == Material.Light ?
    Material.color(Material.Grey, Material.Shade100) :
    Material.color(Material.Grey, Material.Shade800)
  Layout.minimumWidth: 320
  Layout.minimumHeight: 300
  anchors.fill: parent

  ScrollView {
    anchors.fill: parent
    TextArea {
      text: ReadOnlyInspector.properties
      readOnly: true
      selectByMouse: true
      wrapMode: Text.Wrap
      font.family: "monospace"
      padding: 12
    }
  }
}

