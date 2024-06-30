<template>
  <div id="app">
    <RobotMap />
  </div>
</template>

<script>
import RobotMap from './components/RobotMap.vue';

export default {
  name: 'App',
  components: {
    RobotMap,
  },
};
</script>

<style>
#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
  margin-top: 60px;
}
</style>

