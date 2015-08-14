var modules;
var jsPlumbInstance;  // to make instance globally available
var settings = {
    testvar: "hoi"
};
var modulesInCanvas = {};

$(document).ready(function(){
    $('#connCheck').click(function(){
        send('"connection_check"');
        return false;
    });

    $('#updateModules').click(function(){
        updateStatus('Asking for modules')
        send('"ask_available_modules"');
        return false;
    });

});

function updateStatus(msg) {
    var dt = new Date();
    var time = dt.getHours() + ":" + dt.getMinutes() + ":" + dt.getSeconds();
    $('#statusLabel').html(msg + " at " + time +")");
}

function updateModules(modulelist) {
    updateStatus('Received modules');
    modules = modulelist;
    console.log(modules);
    showModulesInPalette(modules);
    initPalette()
    updateStatus('Updated module panel')
}

function showModulesInPalette(modules) {
    $('#palette #list').html("");
    modules.forEach(function(module){
        $('#palette #list').append('<div class="module_palette" id="'+module.id+'">'+module.id+'</div>');
    });
} 

function initPalette() {
    // $(".module_palette").draggable({revert: "invalid"});
    // $(".project-container").droppable({accept: ".module_palette", drop: function(event, ui){
        // alert('test');
    // }})

    
    $(".module_palette").attr({
        "draggable" : "true",
        // "ondragstart" : "startDrag(event)",
        // "ondragend" : "alert('hey! dit werkt')"
    });

    $(".module_palette").on("dragstart", startDrag);

    $(".project-container").attr({
        "ondragover" : "allowDrop(event)",
        "ondrop" : "dropModule(event)"
    });
    
}

jsPlumb.ready(function () {

    jsPlumbInstance = jsPlumb.getInstance({
        // default drag options
        DragOptions: { cursor: 'pointer', zIndex: 2000 },
        // the overlays to decorate each connection with.  note that the label overlay uses a function to generate the label text; in this
        // case it returns the 'labelText' member that we set on each connection in the 'init' method below.
        ConnectionOverlays: [
            [ "Arrow", { location: 1 } ],
            [ "Label", {
                location: 0.1,
                id: "label",
                cssClass: "aLabel"
            }]
        ],
        Container: "project-container"
    });

    var basicType = {
        connector: "StateMachine",
        paintStyle: { strokeStyle: "red", lineWidth: 4 },
        hoverPaintStyle: { strokeStyle: "blue" },
        overlays: [
            "Arrow"
        ]
    };
    jsPlumbInstance.registerConnectionType("basic", basicType);

    // this is the paint style for the connecting lines..
    var connectorPaintStyle = {
            lineWidth: 4,
            strokeStyle: "#61B7CF",
            joinstyle: "round",
            outlineColor: "white",
            outlineWidth: 2
        },
    // .. and this is the hover style.
        connectorHoverStyle = {
            lineWidth: 4,
            strokeStyle: "#216477",
            outlineWidth: 2,
            outlineColor: "white"
        },
        endpointHoverStyle = {
            fillStyle: "#216477",
            strokeStyle: "#216477"
        },
    // the definition of source endpoints (the small blue ones)
        sourceEndpoint = {
            endpoint: "Dot",
            paintStyle: {
                strokeStyle: "#7AB02C",
                fillStyle: "transparent",
                radius: 7,
                lineWidth: 3
            },
            isSource: true,
            connector: [ "Flowchart", { stub: [40, 60], gap: 10, cornerRadius: 5, alwaysRespectStubs: true } ],
            connectorStyle: connectorPaintStyle,
            hoverPaintStyle: endpointHoverStyle,
            connectorHoverStyle: connectorHoverStyle,
            dragOptions: {},

        },
    // the definition of target endpoints (will appear when the user drags a connection)
        targetEndpoint = {
            endpoint: "Dot",
            paintStyle: { fillStyle: "#7AB02C", radius: 11 },
            hoverPaintStyle: endpointHoverStyle,
            maxConnections: -1,
            dropOptions: { hoverClass: "hover", activeClass: "active" },
            isTarget: true,
            overlays: [
                [ "Label", { location: [0.5, -0.5], label: "Drop", cssClass: "endpointTargetLabel" } ]
            ]
        },
        init = function (connection) {
            connection.getOverlay("label").setLabel(connection.sourceId.substring(15) + "-" + connection.targetId.substring(15));
        };

    //var _addEndpoints = function (toId, sourceAnchors, targetAnchors) {
    //    for (var i = 0; i < sourceAnchors.length; i++) {
    //        var sourceUUID = toId + sourceAnchors[i];
    //        jsPlumbInstance.addEndpoint("flowchart" + toId, sourceEndpoint, {
    //            anchor: sourceAnchors[i], uuid: sourceUUID
    //        });
    //    }
    //    for (var j = 0; j < targetAnchors.length; j++) {
    //        var targetUUID = toId + targetAnchors[j];
    //        jsPlumbInstance.addEndpoint("flowchart" + toId, targetEndpoint, { anchor: targetAnchors[j], uuid: targetUUID });
    //    }
    //};

    var addInputsToModule = function (moduleId, inputs) {
        for (var i = 0; i < inputs.length; i++) {
            var anchorId = moduleId + "input" + i.toString;
            var pos = [0, ((i+1)/(inputs.length +1)), 1, 0];
            jsPlumbInstance.addEndpoint(moduleId, sourceEndpoint, {
                anchor: pos,
                overlays: [
                    [ "Label", {
                        location: [-1.5, 0.5],
                        label: inputs[i],
                        cssClass: "endpointTargetLabel"
                    } ]
                ]
            });
        }
    };

    var addOutputsToModule = function(moduleId, outputs) {
        for (var i = 0; i < outputs.length; i++) {
            var anchorId = moduleId + "output" + i.toString;
            var pos = [1, ((i+1)/(outputs.length +1)), 1, 0];
            jsPlumbInstance.addEndpoint(moduleId, sourceEndpoint, {
                anchor: pos,
                overlays: [
                    [ "Label", {
                        location: [-1.5, 0.5],
                        label: outputs[i],
                        cssClass: "endpointSourceLabel"
                    } ]
                ]
            });
        }
    };

    // suspend drawing and initialise.
    jsPlumbInstance.batch(function () {

        addOutputsToModule("flowchartWindow1" , ["Output 1", "Output 2"]);
        addInputsToModule("flowchartWindow1" , ["Input 1"]);

        // listen for new connections; initialise them the same way we initialise the connections at startup.
        jsPlumbInstance.bind("connection", function (connInfo, originalEvent) {
            init(connInfo.connection);
        });

        // make all the window divs draggable
        jsPlumbInstance.draggable($(".project-container .window"), { grid: [20, 20] });
        // THIS DEMO ONLY USES getSelector FOR CONVENIENCE. Use your library's appropriate selector
        // method, or document.querySelectorAll:
        //jsPlumb.draggable(document.querySelectorAll(".window"), { grid: [20, 20] });

        // connect a few up
        jsPlumbInstance.connect({uuids: ["Window2BottomCenter", "Window3TopCenter"], editable: true});
        jsPlumbInstance.connect({uuids: ["Window2LeftMiddle", "Window4LeftMiddle"], editable: true});
        jsPlumbInstance.connect({uuids: ["Window4TopCenter", "Window4RightMiddle"], editable: true});
        jsPlumbInstance.connect({uuids: ["Window3RightMiddle", "Window2RightMiddle"], editable: true});
        jsPlumbInstance.connect({uuids: ["Window4BottomCenter", "Window1TopCenter"], editable: true});
        jsPlumbInstance.connect({uuids: ["Window3BottomCenter", "Window1BottomCenter"], editable: true});
        //

        //
        // listen for clicks on connections, and offer to delete connections on click.
        //
        jsPlumbInstance.bind("click", function (conn, originalEvent) {
            // if (confirm("Delete connection from " + conn.sourceId + " to " + conn.targetId + "?"))
            //   instance.detach(conn);
            conn.toggleType("basic");
        });

        jsPlumbInstance.bind("connectionDrag", function (connection) {
            console.log("connection " + connection.id + " is being dragged. suspendedElement is ", connection.suspendedElement, " of type ", connection.suspendedElementType);
        });

        jsPlumbInstance.bind("connectionDragStop", function (connection) {
            console.log("connection " + connection.id + " was dragged");
        });

        jsPlumbInstance.bind("connectionMoved", function (params) {
            console.log("connection " + params.connection.id + " was moved");
        });
    });

    jsPlumb.fire("jsPlumbDemoLoaded", jsPlumbInstance);

});

function startDrag(ev) {
    console.log(ev);
    ev = ev.originalEvent;
    updateStatus("dragging object: " + ev.target.id);
    ev.dataTransfer.setData("moduleId", ev.target.id);
}

function allowDrop(ev) {
    // updateStatus("allowing drop now..");
    ev.preventDefault();
}

function dropModule(ev) {
    updateStatus("dropped a module to the project-container");
    ev.preventDefault();

    // module id:
    // TODO: make unique
    var data = ev.dataTransfer.getData("moduleId");
    var modId = data;

    
    $(".project-container").append('<div class="window" id="'+modId+'"><strong>'+modId+'</strong><br/><br/></div>');
    
    // make draggable
    var instance = jsPlumbInstance;
    instance.draggable($(".project-container .window"), { grid: [20, 20] });

    // start module at correct position
    var width = $(".project-container .window").width();
    var height = $(".project-container .window").height();
    // TODO: adjust for when not gripping in the center

    var X = ev.pageX - 0.5*width;
    var Y = ev.pageY - 0.5*width;

    $(".project-container #"+modId).offset({
        top : Y,
        left: X
    });


    console.log(settings.testvar);
    // connections
    var sourceUUID = modId + "TopCenter";
    // instance.addEndpoint("flowchart" + modId,
        // sourceEndpoint, {anchor: "TopCenter", uuid: sourceUUID});

    // for (var i = 0; i < sourceAnchors.length; i++) {
    //         var sourceUUID = toId + sourceAnchors[i];
    //         instance.addEndpoint("flowchart" + toId, sourceEndpoint, {
    //             anchor: sourceAnchors[i], uuid: sourceUUID
    //         });
    //     }
    //     for (var j = 0; j < targetAnchors.length; j++) {
    //         var targetUUID = toId + targetAnchors[j];
    //         instance.addEndpoint("flowchart" + toId, targetEndpoint, { anchor: targetAnchors[j], uuid: targetUUID });
    //     }
}
