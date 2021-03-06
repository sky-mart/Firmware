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
import QtQuick.Controls 1.4

import QGroundControl.Controls 1.0
import QGroundControl.FactSystem 1.0
import QGroundControl.FactControls 1.0
import QGroundControl.Controllers 1.0
import "."

FactPanel {
    function pad(width, string, padding)
    {
        return (width <= string.length) ? string : pad(width, string + padding, padding)
    }
    
    id: panel
    
    property var qgcView: null // Temporary hack for broken QGC parameter validation implementation

    CustomCommandWidgetController { id: controller; factPanel: panel }

    // Your own custom changes start here - everything else above is always required

    Column {
        GroupBox {
            title: "Mode"

            Row {
                ExclusiveGroup { id: modeGroup }

                QGCRadioButton {
                    text: "SILENCE"
                    checked: true
                    exclusiveGroup: modeGroup
                    onClicked: controller.sendCommand(10001, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0)
                }

                QGCRadioButton {
                    text: "MANUAL"
                    exclusiveGroup: modeGroup
                    onClicked: controller.sendCommand(10001, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0)
                }

                QGCRadioButton {
                    text: "PID"
                    exclusiveGroup: modeGroup
                    onClicked: controller.sendCommand(10001, 1, 0, 2, 0, 0, 0, 0, 0, 0, 0)
                }

                QGCRadioButton {
                    text: "IMPULSE"
                    exclusiveGroup: modeGroup
                    onClicked: controller.sendCommand(10001, 1, 0, 3, 0, 0, 0, 0, 0, 0, 0)
                }

                QGCRadioButton {
                    text: "SIN"
                    exclusiveGroup: modeGroup
                    onClicked: controller.sendCommand(10001, 1, 0, 4, 0, 0, 0, 0, 0, 0, 0)
                }

                QGCRadioButton {
                    text: "TWO"
                    exclusiveGroup: modeGroup
                    onClicked: controller.sendCommand(10001, 1, 0, 5, 0, 0, 0, 0, 0, 0, 0)
                }
            }  
        }

        GroupBox {
            title: "PWM"

            Grid {
                rows: 3
                columns: 3
                spacing: 3

                QGCLabel {
                    id: left
                    text: "Left"
                }

                Slider {
                    id: pwm1
                    minimumValue: -1.0
                    maximumValue: 1.0
                    value: -1.0
                    onValueChanged: left.text = pad(12, "Left: " + value.toFixed(2), ' ');
                }

                QGCButton {
                    text: "Send"
                    onClicked: controller.sendCommand(10002, 1, 0, 0, pwm1.value, 0, 0, 0, 0, 0, 0)
                }

                QGCLabel {
                    id: right
                    text: "Right"
                }

                Slider {
                    id: pwm2
                    minimumValue: -1.0
                    maximumValue: 1.0
                    value: -1.0
                    onValueChanged: right.text = pad(12, "Right: " + value.toFixed(2), ' ')
                }

                QGCButton {
                    text: "Send"
                    onClicked: controller.sendCommand(10002, 1, 0, 1, pwm2.value, 0, 0, 0, 0, 0, 0)
                }

                QGCLabel {
                    id: back
                    text: "Back"
                }

                Slider {
                    id: pwm3
                    minimumValue: -1.0
                    maximumValue: 1.0
                    value: -1.0
                    onValueChanged: back.text = pad(12, "Back: " + value.toFixed(2), ' ')
                }

                QGCButton {
                    text: "Send"
                    onClicked: {
                        controller.sendCommand(10002, 1, 0, 0, pwm1.value, 0, 0, 0, 0, 0, 0);
                        controller.sendCommand(10002, 1, 0, 1, pwm2.value, 0, 0, 0, 0, 0, 0);
                    }
                }
            }
        }

        GroupBox {
            title: "PID"

            Grid {
                rows: 2
                columns: 4
                spacing: 3 

                QGCLabel { text: "P" }

                QGCLabel { text: "I" }

                QGCLabel { text: "D" }

                QGCLabel {
                    text: ""
                    width: 1
                    height: 1
                }

                QGCTextField {
                    id: p
                    text: "1"
                }

                QGCTextField {
                    id: i
                    text: "0"
                }

                QGCTextField {
                    id: d
                    text: "0"
                }

                QGCButton {
                    text: "Send"
                    onClicked: controller.sendCommand(10003, 1, 0, p.text, i.text, d.text, 0, 0, 0, 0)
                }
            }
        }

        GroupBox {
            title: "impulse"

            Grid {
                rows: 2
                columns: 3
                spacing: 3

                QGCLabel { text: "length"}

                QGCLabel { 
                    id: imp_ampl_display
                    text: "amplitude"
                }

                QGCLabel {
                    text: ""
                    width: 1
                    height: 1
                }

                QGCTextField {
                    id: imp_len
                    text: "20"
                }

                Slider {
                    id: imp_ampl
                    minimumValue: -1.0
                    maximumValue: 1.0
                    value: 1.0
                    onValueChanged: imp_ampl_display.text = "Amplitude: " + value
                }

                QGCButton {
                    text: "Send"
                    onClicked: controller.sendCommand(10004, 1, 0, imp_len.text, imp_ampl.value, 0, 0, 0, 0, 0)
                }
            }
        }

        GroupBox {
            title: "Anchor"
            Row { 
                QGCTextField {
                    id: anchor_len
                    text: "100"
                }

                QGCButton {
                    text: "Send"
                    onClicked: controller.sendCommand(10005, 1, 0, anchor_len.text, 0, 0, 0, 0, 0, 0)
                }

            }
        }

        GroupBox {
            title: "min control"
            Row {
                QGCTextField {
                    id: min_control
                    text: "-1.0"
                }

                QGCButton {
                    text: "Send"
                    onClicked: controller.sendCommand(10006, 1, 0, min_control.text, 0, 0, 0, 0, 0, 0)
                }

            }
        }

        GroupBox {
            title: "sin"
            Row {
                QGCTextField {
                    id: sin_mag
                    text: "0.1"
                }

                QGCTextField {
                    id: sin_freq
                    text: "1.0"
                }

                QGCButton {
                    text: "Send"
                    onClicked: controller.sendCommand(10007, 1, 0, sin_mag.text, sin_freq.text, 0, 0, 0, 0, 0)
                }

            }
        }

        GroupBox {
            title: "two"
            Row {
                QGCTextField {
                    id: tau
                    text: "0.1"
                }

                QGCTextField {
                    id: max_step
                    text: "0.1"
                }

                QGCButton {
                    text: "Send"
                    onClicked: controller.sendCommand(10008, 1, 0, tau.text, max_step.text, 0, 0, 0, 0, 0)
                }

            }
        }
    }

    // Grid {
    //     id: paramsGrid;

    //     rows: 8 
    //     columns: 2
    //     spacing: 3
    //     // The QGCButton control is provided by QGroundControl.Controls. It is a wrapper around
    //     // the standard Qml Button element which using the default QGC font and color palette.

        

    //     // The FactTextField control is provides by GroundControl.FactControls. It is a wrapper
    //     // around the Qml TextField element which allows you to bind it directly to any parameter.
    //     // The parameter is changed automatically when you click enter or click away from the field.
    //     // Understand that there is currently no value validation. So you may crash your vehicle by
    //     // setting a parameter to an incorrect value. Validation will come in the future.

    //     // Be very careful when referencing parameters. If you specify a parameter which does not exist
    //     // QGroundControl will warn and shutdown.

    //     QGCLabel {
    //         id: cmd_label
    //         text: "Command ID"
    //     }

    //     QGCLabel {
    //         id: param4_label
    //         text: "Param 4"
    //     }

    //     QGCTextField {
    //         onEditingFinished: panel.cmd_id = text
    //     }

    //     QGCTextField {
    //         onEditingFinished: {
    //             panel.param4 = text
    //             param4_label.text = panel.param4
    //         }
    //     }

    //     QGCLabel {
    //         id: param1_label
    //         text: "Param 1"
    //     }

    //     QGCLabel {
    //         id: param5_label
    //         text: "Param 5"
    //     }

    //     QGCTextField {
    //         onEditingFinished: {
    //             panel.param1 = text
    //             param1_label.text = panel.param1
    //         }
    //     }

    //     QGCTextField {
    //         onEditingFinished: {
    //             panel.param5 = text
    //             param5_label.text = panel.param5
    //         }
    //     }

    //     QGCLabel {
    //         id: param2_label
    //         text: "Param 2"
    //     }

    //     QGCLabel {
    //         id: param6_label
    //         text: "Param 6"
    //     }

    //     QGCTextField {
    //         onEditingFinished: {
    //             panel.param2 = text
    //             param2_label.text = panel.param2
    //         }
    //     }

    //     QGCTextField {
    //         onEditingFinished: {
    //             panel.param6 = text
    //             param6_label.text = panel.param6
    //         }
    //     }

    //     QGCLabel {
    //         id: param3_label
    //         text: "Param 3"
    //     }

    //     QGCLabel {
    //         id: param7_label
    //         text: "Param 7"
    //     }

    //     QGCTextField {
    //         onEditingFinished: {
    //             panel.param3 = text
    //             param3_label.text = panel.param3
    //         }
    //     }

    //     QGCTextField {
    //         onEditingFinished: {
    //             panel.param7 = text
    //             param7_label.text = panel.param7
    //         }
    //     }

    //     // FactTextField {
    //     //     // The -1 signals default component id.
    //     //     // You can replace it with a specific component id if you like
    //     //     fact: controller.getParameterFact(-1, "MAV_SYS_ID")
    //     // }
    // }

    // QGCButton {
    //     text: "Send Command"
    //     onClicked: controller.sendCommand(panel.cmd_id, 1, 0,
    //         panel.param1, panel.param2, panel.param3, panel.param4, panel.param5, panel.param6, panel.param7)

    //     anchors.left: parent.left
    //     anchors.bottom: parent.bottom
    // }

    // Your own custom changes end here - everything else below is always required
}