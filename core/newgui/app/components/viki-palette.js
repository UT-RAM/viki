import Ember from 'ember';

export default Ember.Component.extend({
  moduleBag: Ember.inject.service(),

  willRender() {
    //this.set('modules', this.get('moduleBag').get('modules'));
    this.set('modules', this.get('moduleBag').get('modules'));
  }

});


