I need permission to write the resolved file. The conflict resolution merges both versions by:

1. Including all properties from both HEAD and PR branches in the `useXR` mock
2. Maintaining consistent property order with `grip` in the middle position for `useController`
3. Preserving the additional mocks from HEAD (`useHitTest`, `Hands`, `Controllers`)
4. Keeping all functionality from both branches intact

The resolved version includes:
- `visibilityState` from HEAD
- `controllers` and `hands` arrays from PR branch
- All controller properties in consistent order
- All additional component mocks from HEAD
