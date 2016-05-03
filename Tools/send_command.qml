// This is an example Custom Command Qml file. You have full access to the entire Qml language
// for creating any user interface you like. From the ui you can affect the following changes
// with respect to your vehicle:
//    1) Sending COMMAND_LONG commands out over mavlink using QGCButton control
//    2) Modifying parameters
//
// When developing custom Qml file implementations. You must restart QGroundControl to pick up
// the changes. You need to do this even if you select Clear Qml file. Not sure what at the this
// point. Qt must be caching the files somewhere.

import QtQuick 2.2

import QGroundControl.Controls 1.0
import QGroundControl.FactSystem 1.0
import QGroundControl.FactControls 1.0
import QGroundControl.Controllers 1.0

FactPanel {
    id: panel
    
    property var qgcView: null // Temporary hack for broken QGC parameter validation implementation

    CustomCommandWidgetController { id: controller; factPanel: panel }

    property var cmd_id
    property var param1
    property var param2
    property var param3
    property var param4
    property var param5
    property var param6
    property var param7

    // Your own custom changes start here - everything else above is always required

    Grid {
        id: paramsGrid;

        rows: 8 
        columns: 2
        spacing: 3
        // The QGCButton control is provided by QGroundControl.Controls. It is a wrapper around
        // the standard Qml Button element which using the default QGC font and color palette.

        

        // The FactTextField control is provides by GroundControl.FactControls. It is a wrapper
        // around the Qml TextField element which allows you to bind it directly to any parameter.
        // The parameter is changed automatically when you click enter or click away from the field.
        // Understand that there is currently no value validation. So you may crash your vehicle by
        // setting a parameter to an incorrect value. Validation will come in the future.

        // Be very careful when referencing parameters. If you specify a parameter which does not exist
        // QGroundControl will warn and shutdown.

        QGCLabel {
            id: cmd_label
            text: "Command ID"
        }

        QGCLabel {
            id: param4_label
            text: "Param 4"
        }

        QGCTextField {
            onEditingFinished: panel.cmd_id = text
        }

        QGCTextField {
            onEditingFinished: {
                panel.param4 = text
                param4_label.text = panel.param4
            }
        }

        QGCLabel {
            id: param1_label
            text: "Param 1"
        }

        QGCLabel {
            id: param5_label
            text: "Param 5"
        }

        QGCTextField {
            onEditingFinished: {
                panel.param1 = text
                param1_label.text = panel.param1
            }
        }

        QGCTextField {
            onEditingFinished: {
                panel.param5 = text
                param5_label.text = panel.param5
            }
        }

        QGCLabel {
            id: param2_label
            text: "Param 2"
        }

        QGCLabel {
            id: param6_label
            text: "Param 6"
        }

        QGCTextField {
            onEditingFinished: {
                panel.param2 = text
                param2_label.text = panel.param2
            }
        }

        QGCTextField {
            onEditingFinished: {
                panel.param6 = text
                param6_label.text = panel.param6
            }
        }

        QGCLabel {
            id: param3_label
            text: "Param 3"
        }

        QGCLabel {
            id: param7_label
            text: "Param 7"
        }

        QGCTextField {
            onEditingFinished: {
                panel.param3 = text
                param3_label.text = panel.param3
            }
        }

        QGCTextField {
            onEditingFinished: {
                panel.param7 = text
                param7_label.text = panel.param7
            }
        }

        // FactTextField {
        //     // The -1 signals default component id.
        //     // You can replace it with a specific component id if you like
        //     fact: controller.getParameterFact(-1, "MAV_SYS_ID")
        // }
    }

    QGCButton {
        text: "Send Command"
        onClicked: controller.sendCommand(panel.cmd_id, 1, 0,
            panel.param1, panel.param2, panel.param3, panel.param4, panel.param5, panel.param6, panel.param7)

        anchors.left: parent.left
        anchors.bottom: parent.bottom
    }

    // Your own custom changes end here - everything else below is always required
}