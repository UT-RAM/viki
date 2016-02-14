import Ember from 'ember';

export default Ember.Service.extend({
    jsplumb: Ember.inject.service(),

    history: [],
    future: [],

    modules_in_canvas: [],
    current_dragging_module: null,

    addModuleToCanvas(module, _x, _y) {
        var uModId;
        uModId = module.id + this.getUniqueGuid(); // TODO: Add the GID part

        // Extra parameters to initialize for later use
        module.uWindowId = uModId;
        module.params = [];  // premake list for parameters
        module.args = [];  // placeholder for command line arguments
        module.prefixes = [];  // placeholder for launch-prefixes
        module.selectedMachines = [];

        // start module at correct position
        var width = 100;
        var height = 20 + Math.max(module.inputs.length, module.outputs.length) * 20;
        var X = Math.floor((_x - 0.5*width) / 20) * 20;
        var Y = Math.floor((_y - 0.5*height) / 20) * 20;

        module.drawInfo = {x: X, y: Y, width: width, height: height};

        this.get('modules_in_canvas').addObject(module);
        this.get('jsplumb').jsPlumbifyModule(module);
    },

    /**
     * Generate a unique id for a module.
     * the id is saved in a list, so not even accidentally id's can be the same
     * @returns a unique id
     */
    getUniqueGuid() {
        //TODO: Fix this!
        // generate until a not-used number has been found
        //do {
          var generatedId = "_" + ("" + (Math.random()*Math.pow(36,4) << 0).toString(36)).slice(-4);
        //}
        //while(generatedGUIDs.indexOf(generatedId) > -1)

        // save number in list
        //generatedGUIDs.push(generatedId);

        return generatedId;
    }
});
