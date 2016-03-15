import Ember from 'ember';

export default Ember.Service.extend({

  modules: [],

  init() {
      this.set('modules', JSON.parse(VikiBackend.getModules()));
  },

  getModuleById(Id) {
      var mods = this.get('modules');
      for (var i=0; i <mods.length; i++) {
          if (mods[i].id == Id) {
              return $.extend(true, {}, mods[i]);
          }
      }
  }
});
