import Ember from 'ember';

export default Ember.Component.extend({

    click() {
      // Click handler to re-fly-in the VIKI logo
      var el     = $("#logo"),
        newone = el.clone(true);

      el.before(newone);
      el.remove();
    }
});
